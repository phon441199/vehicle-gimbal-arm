/**
 * motor_test_esp32.ino  -  LOLIN D32 (ESP32)
 * GM4108 x2 bring-up on MKS DUAL FOC, SimpleFOC. Encoders on the verified
 * shared SPI bus (CS 21/22). No IMU, no CAN.
 *
 * ---- ESP32 -> MKS wiring ----------------------------------------------------
 *   M0 PWM A/B/C = 25/26/27   EN0 = 14
 *   M1 PWM A/B/C = 16/17/4    EN1 = 12   (strapping: add 10k pulldown if boot fails)
 *   Current  M0 = 36/39   M1 = 34/32     (ADC1)
 *   Encoder SPI: SCK=18 MISO=19 MOSI=23   ENC1_CS=21  ENC2_CS=22
 *   GND -> MKS GND (star ground; motor power GND separate heavy wire to PSU-)
 * ----------------------------------------------------------------------------
 *
 * Bring-up: PSU 12V, CURRENT LIMIT ~0.5A first. Boot does FOC align (twitch).
 * Serial @115200. Keys:
 *   3 enc check   4 current raw   9 current auto('9' then '1'/'2')
 *   5 spin M0     6 spin M1       x stop   0/? menu
 *
 * Libraries: SimpleFOC
 */

#include <SimpleFOC.h>
#include <SPI.h>

// ---- pins ------------------------------------------------------------------
#define M0_A 25
#define M0_B 26
#define M0_C 27
#define M0_EN 14
#define M1_A 16
#define M1_B 17
#define M1_C 4
#define M1_EN 12

#define CUR0_A 36
#define CUR0_B 39
#define CUR1_A 34
#define CUR1_B 32

#define PIN_SCK 18
#define PIN_MISO 19
#define PIN_MOSI 23
#define ENC1_CS 21
#define ENC2_CS 22
#define IMU_CS  13   // IMU on same bus: keep deselected (HIGH) or it corrupts SPI

// ---- params ----------------------------------------------------------------
#define GM4108_PP       11
#define GM4108_R        5.5f
#define SUPPLY_V        12.0f
#define DRIVER_VLIMIT   8.0f
#define MOTOR_VLIMIT    5.0f
#define MOTOR_ILIMIT    1.0f
#define MOTOR_VEL_LIMIT 20.0f
#define CS_SHUNT        0.01f
#define CS_GAIN         50.0f
#define SPIN_VEL        5.0f
#define OL_VEL          10.0f    // open-loop test speed
#define OL_VLIMIT       3.0f     // open-loop voltage cap (no back-EMF ctrl)
#define ENC_SPI_HZ      1000000UL

MagneticSensorSPIConfig_s AS5048_CFG = {
  .spi_mode = SPI_MODE1, .clock_speed = ENC_SPI_HZ, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_CFG, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_CFG, ENC2_CS);

BLDCMotor motor1 = BLDCMotor(GM4108_PP, GM4108_R);
BLDCMotor motor2 = BLDCMotor(GM4108_PP, GM4108_R);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M0_A, M0_B, M0_C, M0_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M1_A, M1_B, M1_C, M1_EN);
InlineCurrentSense cs1 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CUR0_A, CUR0_B);
InlineCurrentSense cs2 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CUR1_A, CUR1_B);

bool m1_ok = false, m2_ok = false;

enum Mode { MENU, ENC_CHECK, CURR_CHECK, CURR_AUTO, SPIN_M0, SPIN_M1, OL_M0, OL_M1 };
Mode mode = MENU;
unsigned long last_ms = 0;

bool encoder_alive(int cs) {
  for (int t = 0; t < 4; t++) {
    SPI.beginTransaction(SPISettings(ENC_SPI_HZ, MSBFIRST, SPI_MODE1));
    digitalWrite(cs, LOW);
    uint16_t raw = SPI.transfer16(0xFFFF);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    if (raw != 0xFFFF && raw != 0x0000) return true;
    delayMicroseconds(100);
  }
  return false;
}

bool init_motor(BLDCMotor &m, int cs, int num) {
  m.init();
  if (!encoder_alive(cs)) {
    Serial.printf("M%d: ENC(CS=%d) no response -- skip FOC\n", num, cs);
    return false;
  }
  for (int i = 0; i < 5; i++) {
    m.initFOC();
    if (m.sensor_direction != Direction::UNKNOWN && m.zero_electric_angle > -1000.0f) {
      Serial.printf("M%d FOC ok: zero=%.4f dir=%d\n", num, m.zero_electric_angle, m.sensor_direction);
      return true;
    }
    Serial.printf("M%d initFOC FAIL retry %d/5\n", num, i + 1);
    delay(300);
  }
  Serial.printf("M%d FOC GAVE UP\n", num);
  return false;
}

void setup_motors() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
  pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);
  pinMode(IMU_CS,  OUTPUT); digitalWrite(IMU_CS,  HIGH);  // deselect IMU on shared bus
  encoder1.init(&SPI);
  encoder2.init(&SPI);
  motor1.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);

  driver1.voltage_power_supply = SUPPLY_V; driver1.voltage_limit = DRIVER_VLIMIT; driver1.init();
  driver2.voltage_power_supply = SUPPLY_V; driver2.voltage_limit = DRIVER_VLIMIT; driver2.init();
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  cs1.linkDriver(&driver1); cs1.init();
  cs1.gain_a *= -1; cs1.gain_b *= -1; cs1.skip_align = true;
  cs2.linkDriver(&driver2); cs2.init();
  cs2.gain_a *= -1; cs2.gain_b *= -1; cs2.skip_align = true;
  motor1.linkCurrentSense(&cs1);
  motor2.linkCurrentSense(&cs2);

  for (BLDCMotor *m : {&motor1, &motor2}) {
    m->voltage_limit = MOTOR_VLIMIT;
    m->current_limit = MOTOR_ILIMIT;
    m->velocity_limit = MOTOR_VEL_LIMIT;
    m->voltage_sensor_align = 5.0f;
    m->controller = MotionControlType::velocity;
    m->torque_controller = TorqueControlType::voltage;
    m->PID_velocity.P = 0.2f;
    m->PID_velocity.I = 2.0f;
    m->PID_velocity.D = 0.0f;
    m->LPF_velocity.Tf = 0.01f;
  }

  m1_ok = init_motor(motor1, ENC1_CS, 1);
  m2_ok = init_motor(motor2, ENC2_CS, 2);
  motor1.disable();
  motor2.disable();
}

// Re-run offset calibration with MKS powered + motors off.
// init() re-averages the idle ADC as the zero; re-apply gain reversal after.
void recal_cs(InlineCurrentSense &cs, int num) {
  cs.init();
  cs.gain_a *= -1; cs.gain_b *= -1;
  cs.skip_align = true;
  PhaseCurrent_s c = cs.getPhaseCurrents();
  Serial.printf("  M%d recal done -> rest Ia=%.3f Ib=%.3f (want ~0)\n", num, c.a, c.b);
}

void do_recal() {
  motor1.disable(); motor2.disable();
  Serial.println("--- recal current offsets (motors OFF, MKS must be POWERED) ---");
  recal_cs(cs1, 1);
  recal_cs(cs2, 2);
}

void do_current_auto(int idx) {
  BLDCDriver3PWM &drv = (idx == 0) ? driver1 : driver2;
  InlineCurrentSense &cs = (idx == 0) ? cs1 : cs2;
  const float Vt = 2.0f;
  Serial.printf("--- Current AUTO M%d (Vt=%.1f) ---\n", idx + 1, Vt);
  drv.enable();
  drv.setPwm(0, 0, 0); delay(80);
  double oa = 0, ob = 0; for (int i = 0; i < 50; i++) { PhaseCurrent_s c = cs.getPhaseCurrents(); oa += c.a; ob += c.b; delayMicroseconds(200);} oa /= 50; ob /= 50;
  drv.setPwm(Vt, 0, 0); delay(150);
  double aa = 0, ab = 0; for (int i = 0; i < 50; i++) { PhaseCurrent_s c = cs.getPhaseCurrents(); aa += c.a; ab += c.b; delayMicroseconds(200);} aa /= 50; ab /= 50;
  drv.setPwm(0, 0, 0); drv.disable();
  Serial.printf("  offset Ia=%.3f Ib=%.3f | driveA Ia=%.3f Ib=%.3f\n", oa, ob, aa, ab);
  double da = aa - oa, db = ab - ob;
  bool pass = fabs(oa) < 0.15 && fabs(ob) < 0.15 && fabs(da) > 0.05 && fabs(da) > fabs(db);
  Serial.printf("  RESULT M%d: %s\n", idx + 1, pass ? "PASS" : "FAIL -- check shunt/gain/wiring");
}

void print_menu() {
  Serial.println("\n===== GM4108 Motor Test (ESP32/D32) =====");
  Serial.printf (" M0(FOC)=%s  M1(FOC)=%s\n", m1_ok ? "OK" : "FAIL", m2_ok ? "OK" : "FAIL");
  Serial.println(" 3 enc check   4 current raw   9 current auto('9'->'1'/'2')");
  Serial.println(" c recal current offsets (MKS powered, motors off)");
  Serial.println(" 5 spin M0     6 spin M1   (closed loop)");
  Serial.println(" a OL M0       b OL M1     (open loop - phase/mech check)");
  Serial.println(" x stop   0/? menu");
  Serial.println("=========================================");
}

void enter(Mode m) {
  motor1.disable(); motor2.disable();
  mode = m;
  switch (m) {
    case ENC_CHECK:
      Serial.printf("ENC1 %s  ENC2 %s\n",
        encoder_alive(ENC1_CS) ? "ALIVE" : "NO RESP",
        encoder_alive(ENC2_CS) ? "ALIVE" : "NO RESP");
      break;
    case CURR_CHECK: Serial.println("--- current at rest (~0A expected, x=stop) ---"); break;
    case CURR_AUTO:  Serial.println("--- press 1/2 to inject & verify, x=menu ---"); break;
    case SPIN_M0:
      if (!m1_ok) { Serial.println("M0 FOC not OK"); mode = MENU; print_menu(); break; }
      Serial.printf("--- spin M0 %.1f rad/s closed (x=stop) ---\n", SPIN_VEL);
      motor1.controller = MotionControlType::velocity; motor1.voltage_limit = MOTOR_VLIMIT;
      motor1.enable(); motor1.target = SPIN_VEL; break;
    case SPIN_M1:
      if (!m2_ok) { Serial.println("M1 FOC not OK"); mode = MENU; print_menu(); break; }
      Serial.printf("--- spin M1 %.1f rad/s closed (x=stop) ---\n", SPIN_VEL);
      motor2.controller = MotionControlType::velocity; motor2.voltage_limit = MOTOR_VLIMIT;
      motor2.enable(); motor2.target = SPIN_VEL; break;
    case OL_M0:
      Serial.printf("--- OPEN-LOOP M0 %.1f rad/s @%.1fV (x=stop) ---\n", OL_VEL, OL_VLIMIT);
      motor1.controller = MotionControlType::velocity_openloop; motor1.voltage_limit = OL_VLIMIT;
      motor1.enable(); motor1.target = OL_VEL; break;
    case OL_M1:
      Serial.printf("--- OPEN-LOOP M1 %.1f rad/s @%.1fV (x=stop) ---\n", OL_VEL, OL_VLIMIT);
      motor2.controller = MotionControlType::velocity_openloop; motor2.voltage_limit = OL_VLIMIT;
      motor2.enable(); motor2.target = OL_VEL; break;
    default: break;
  }
}

void key(char c) {
  if (c == '0' || c == '?') { mode = MENU; print_menu(); return; }
  if (c == 'x' || c == 'X') { Serial.println(">> stop"); enter(MENU); print_menu(); return; }
  if (mode == CURR_AUTO && (c == '1' || c == '2')) {
    do_current_auto(c == '2' ? 1 : 0); enter(MENU); print_menu(); return;
  }
  switch (c) {
    case '3': enter(ENC_CHECK); break;
    case '4': enter(CURR_CHECK); break;
    case '9': enter(CURR_AUTO); break;
    case 'c': case 'C': do_recal(); mode = MENU; print_menu(); break;
    case '5': enter(SPIN_M0); break;
    case '6': enter(SPIN_M1); break;
    case 'a': case 'A': enter(OL_M0); break;
    case 'b': case 'B': enter(OL_M1); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n### GM4108 Motor Test (ESP32/D32) ###");
  Serial.println("PSU 12V, CURRENT LIMIT ~0.5A first. Motors will twitch (FOC align).");
  setup_motors();
  print_menu();
}

void loop() {
  encoder1.update();
  encoder2.update();
  if (motor1.enabled) {
    if (motor1.controller != MotionControlType::velocity_openloop) motor1.loopFOC();
    motor1.move();
  }
  if (motor2.enabled) {
    if (motor2.controller != MotionControlType::velocity_openloop) motor2.loopFOC();
    motor2.move();
  }

  while (Serial.available()) key(Serial.read());

  if (millis() - last_ms >= 200) {
    last_ms = millis();
    switch (mode) {
      case ENC_CHECK:
        Serial.printf("  enc1=%.3f enc2=%.3f rad\n", encoder1.getAngle(), encoder2.getAngle());
        break;
      case CURR_CHECK: {
        PhaseCurrent_s a = cs1.getPhaseCurrents();
        PhaseCurrent_s b = cs2.getPhaseCurrents();
        Serial.printf("  M0 Ia=%.3f Ib=%.3f | M1 Ia=%.3f Ib=%.3f [A]\n", a.a, a.b, b.a, b.b);
        break;
      }
      case SPIN_M0:
        Serial.printf("  M0 vel=%.2f (tgt %.1f) ang=%.2f\n",
          motor1.shaft_velocity, SPIN_VEL, encoder1.getAngle());
        break;
      case SPIN_M1:
        Serial.printf("  M1 vel=%.2f (tgt %.1f) ang=%.2f\n",
          motor2.shaft_velocity, SPIN_VEL, encoder2.getAngle());
        break;
      case OL_M0:
        Serial.printf("  [OL] M0 enc_ang=%.2f (should climb steadily if turning)\n",
          encoder1.getAngle());
        break;
      case OL_M1:
        Serial.printf("  [OL] M1 enc_ang=%.2f (should climb steadily if turning)\n",
          encoder2.getAngle());
        break;
      default: break;
    }
  }
}
