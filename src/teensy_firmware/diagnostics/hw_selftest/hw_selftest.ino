/**
 * hw_selftest.ino  -  Teensy 4.1
 * One-firmware hardware self-test for the GimbalArm rig.
 * Press a key over Serial @115200 to run each check; press 'x' to stop and
 * return to the menu. Nothing moves until you select a motor test.
 *
 * Covers:
 *   1  I2C scan            - find ICM20948 (expect 0x68)
 *   2  IMU live            - stream accel / gyro
 *   3  Encoder check       - AS5048A x2 alive + angle (CS9 / CS10)
 *   4  Current raw         - InlineCurrentSense phase currents at rest (offset health)
 *   5  Gimbal M1 spin      - GM4108 #1 closed-loop velocity (full FOC chain)
 *   6  Gimbal M2 spin      - GM4108 #2 closed-loop velocity
 *   7  AK40 status         - stream CAN status (theta / omega) for ids 1,2
 *   8  AK40 PID hold       - AK40 impedance hold at current pose (id selectable)
 *
 * Pins / params copied 1:1 from gimbal_arm_full.ino (single source of truth).
 *   CAN1:  CTX=22 CRX=23   GM4108 M1: PWM 0/1/2 EN 3   M2: PWM 4/5/6 EN 7
 *   ENC:   CS1=9 CS2=10  SPI MOSI=11 MISO=12 SCK=13   IMU: I2C0 SDA=18 SCL=19
 *   Current: CS1 A0/A1  CS2 A2/A3
 *
 * Libraries: FlexCAN_T4, SimpleFOC, SparkFun ICM_20948
 */

#include <FlexCAN_T4.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM_20948.h>

// ============================================================================
// Pins / params  (identical to gimbal_arm_full.ino)
// ============================================================================
#define WM1_PWM_A 0
#define WM1_PWM_B 1
#define WM1_PWM_C 2
#define WM1_EN    3
#define WM2_PWM_A 4
#define WM2_PWM_B 5
#define WM2_PWM_C 6
#define WM2_EN    7
#define ENC1_CS   9
#define ENC2_CS   10
#define CS1_A     A0
#define CS1_B     A1
#define CS2_A     A2
#define CS2_B     A3

#define GM4108_PP        11
#define GM4108_PHASE_R   5.5f
#define SUPPLY_VOLTAGE   12.0f
#define DRIVER_VLIMIT    8.0f
#define MOTOR_VLIMIT     5.0f
#define MOTOR_ILIMIT     1.0f
#define MOTOR_VEL_LIMIT  20.0f
#define CS_SHUNT         0.01f
#define CS_GAIN          50.0f
#define SPIN_VEL_RADS    5.0f      // gimbal spin test target speed

// IMU
#define I2C_CLOCK_HZ  500000
#define AD0_VAL       0

// AK40
#define M1_CAN_ID 1
#define M1_KT     0.0705f
#define M2_CAN_ID 2
#define M2_KT     0.0908f
#define GEAR_RATIO 10.0f
#define GEAR_EFF   0.90f
#define POLE_PAIRS 14
#define POS_AT_JOINT 1
#define AK40_NUM  2
#define K_IMP     2.0f
#define D_IMP     0.05f
#define I_MAX     2.0f
#define POS_ERR_LIMIT_DEG 60.0f
#define AK40_CTRL_HZ 200
#define STATUS_PKT 0x29
#define DEG2RAD  0.0174532925f
#define RPM2RADS 0.1047197551f

typedef enum {
  CAN_PACKET_SET_CURRENT     = 1,
  CAN_PACKET_SET_ORIGIN_HERE = 5,
} CAN_PACKET_ID;

// ============================================================================
// Objects
// ============================================================================
BLDCMotor wmotor1 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCMotor wmotor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM wdriver1 = BLDCDriver3PWM(WM1_PWM_A, WM1_PWM_B, WM1_PWM_C, WM1_EN);
BLDCDriver3PWM wdriver2 = BLDCDriver3PWM(WM2_PWM_A, WM2_PWM_B, WM2_PWM_C, WM2_EN);

MagneticSensorSPIConfig_s AS5048_SPI_SLOW = {
  .spi_mode = SPI_MODE1, .clock_speed = 100000, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC2_CS);

InlineCurrentSense current_sense1 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS1_A, CS1_B);
InlineCurrentSense current_sense2 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS2_A, CS2_B);

bool m1_ok = false, m2_ok = false;
float foc_zero1 = 0, foc_zero2 = 0;

ICM_20948_I2C myICM;
bool imu_ok = false;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

struct AK40Motor {
  uint8_t can_id; float Kt, dir;
  float theta, omega; bool haveStatus;
  float theta_d, K, D; bool enabled;
  float omega_f, dbg_I;
};
AK40Motor ak[AK40_NUM];
int ak_sel = 0;   // 0=id1, 1=id2

// ============================================================================
// Mode machine
// ============================================================================
enum Mode { MENU, I2C_SCAN, IMU_LIVE, ENC_CHECK, CURR_CHECK, CURR_SELFTEST,
            GIMBAL_M1, GIMBAL_M2, AK40_STATUS, AK40_HOLD };
Mode mode = MENU;
String serial_buf = "";
unsigned long last_print_ms = 0;

// ============================================================================
// Shared init helpers (patterns from gimbal_arm_full.ino)
// ============================================================================
bool encoder_alive(int cs) {
  for (int t = 0; t < 4; t++) {
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
    digitalWrite(cs, LOW);
    uint16_t raw = SPI.transfer16(0xFFFF);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    if (raw != 0xFFFF) return true;
    delayMicroseconds(100);
  }
  return false;
}

bool init_one_motor(BLDCMotor &m, int cs, float &foc_zero, int num) {
  m.init();
  if (!encoder_alive(cs)) {
    Serial.printf("M%d: ENC(CS=%d) no response -- skip initFOC. check wiring\n", num, cs);
    return false;
  }
  for (int i = 0; i < 5; i++) {
    m.initFOC();
    if (m.sensor_direction != Direction::UNKNOWN && m.zero_electric_angle > -1000.0f) {
      foc_zero = m.zero_electric_angle;
      Serial.printf("M%d FOC: zero=%.4f dir=%d (ok)\n", num, foc_zero, m.sensor_direction);
      return true;
    }
    if (!encoder_alive(cs)) { Serial.printf("M%d: ENC dropped during align\n", num); break; }
    Serial.printf("M%d initFOC FAIL, retry %d/5\n", num, i + 1);
    delay(300);
  }
  Serial.printf("M%d FOC: GAVE UP -- sensor dead\n", num);
  return false;
}

void setup_gimbal_motors() {
  encoder1.init();
  encoder2.init();
  wmotor1.linkSensor(&encoder1);
  wmotor2.linkSensor(&encoder2);

  wdriver1.voltage_power_supply = SUPPLY_VOLTAGE; wdriver1.voltage_limit = DRIVER_VLIMIT; wdriver1.init();
  wdriver2.voltage_power_supply = SUPPLY_VOLTAGE; wdriver2.voltage_limit = DRIVER_VLIMIT; wdriver2.init();
  wmotor1.linkDriver(&wdriver1);
  wmotor2.linkDriver(&wdriver2);

  current_sense1.linkDriver(&wdriver1);
  current_sense1.init();
  current_sense1.gain_a *= -1; current_sense1.gain_b *= -1;
  current_sense1.skip_align = true;
  current_sense2.linkDriver(&wdriver2);
  current_sense2.init();
  current_sense2.gain_a *= -1; current_sense2.gain_b *= -1;
  current_sense2.skip_align = true;
  wmotor1.linkCurrentSense(&current_sense1);
  wmotor2.linkCurrentSense(&current_sense2);

  // velocity mode for the spin test (confirms full FOC chain incl. encoder)
  for (BLDCMotor *m : {&wmotor1, &wmotor2}) {
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

  m1_ok = init_one_motor(wmotor1, ENC1_CS, foc_zero1, 1);
  m2_ok = init_one_motor(wmotor2, ENC2_CS, foc_zero2, 2);
  wmotor1.disable();
  wmotor2.disable();
}

bool setup_icm20948() {
  myICM.begin(Wire, AD0_VAL);
  if (myICM.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fss; fss.a = gpm4; fss.g = dps500;
  myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_dlpcfg_t dlpcfg; dlpcfg.a = acc_d50bw4_n68bw8; dlpcfg.g = gyr_d51bw2_n73bw3;
  myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
  myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  ICM_20948_smplrt_t srd; srd.a = 10; srd.g = 10;
  myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
  myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);
  return true;
}

// ============================================================================
// AK40 CAN helpers (from gimbal_arm_full.ino)
// ============================================================================
static void buf_append_int32(uint8_t *b, int32_t v, int *i) {
  b[(*i)++] = v >> 24; b[(*i)++] = v >> 16; b[(*i)++] = v >> 8; b[(*i)++] = v;
}
static int16_t buf_get_int16(const uint8_t *b, int o) {
  return (int16_t)((b[o] << 8) | b[o + 1]);
}
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static void canSend(uint8_t id, CAN_PACKET_ID pkt, const uint8_t *d, uint8_t len) {
  CAN_message_t m;
  m.id = ((uint32_t)pkt << 8) | id;
  m.flags.extended = 1;
  m.len = len;
  for (uint8_t k = 0; k < len; k++) m.buf[k] = d[k];
  Can1.write(m);
}
static void motorSetCurrent(uint8_t id, float amps) {
  uint8_t b[4]; int i = 0;
  buf_append_int32(b, (int32_t)(amps * 1000.0f), &i);
  canSend(id, CAN_PACKET_SET_CURRENT, b, i);
}
static void motorSetOrigin(uint8_t id) {
  uint8_t b[1] = {0};
  canSend(id, CAN_PACKET_SET_ORIGIN_HERE, b, 1);
}
static void parseStatus(const CAN_message_t &msg) {
  if (!msg.flags.extended) return;
  if (((msg.id >> 8) & 0xFF) != STATUS_PKT) return;
  uint8_t id = msg.id & 0xFF;
  if (msg.len != 8) return;
  for (int k = 0; k < AK40_NUM; k++) {
    if (ak[k].can_id != id) continue;
    float pos_deg = buf_get_int16(msg.buf, 0) * 0.1f;
    float erpm    = buf_get_int16(msg.buf, 2) * 10.0f;
    if (!POS_AT_JOINT) pos_deg /= GEAR_RATIO;
    ak[k].theta = pos_deg * DEG2RAD;
    ak[k].omega = (erpm / (float)POLE_PAIRS / GEAR_RATIO) * RPM2RADS;
    ak[k].haveStatus = true;
    return;
  }
}
static void ak_holdStep(AK40Motor &m, float dt) {
  if (!m.enabled || !m.haveStatus) return;
  float th = m.theta, om = m.omega;
  if (fabsf(m.theta_d - th) > POS_ERR_LIMIT_DEG * DEG2RAD) {
    m.enabled = false; motorSetCurrent(m.can_id, 0.0f);
    Serial.printf("!! AK%u POS LIMIT -> STOP\n", m.can_id);
    return;
  }
  float a = 2.0f * 3.14159265f * 25.0f * dt; a = a / (1.0f + a);
  m.omega_f += a * (om - m.omega_f);
  float tau = m.K * (m.theta_d - th) + m.D * (0.0f - m.omega_f);
  float kt_tot = m.Kt * GEAR_RATIO * GEAR_EFF;
  float I = clampf(m.dir * tau / kt_tot, -I_MAX, I_MAX);
  motorSetCurrent(m.can_id, I);
  m.dbg_I = I;
}

// ============================================================================
// Current-sense auto verification
//   Drive one phase with a known small voltage straight through the driver
//   (no FOC), read back phase currents, and check each channel responds with
//   the right sign and a plausible magnitude. Motor twitches briefly.
// ============================================================================
static void readCurrAvg(InlineCurrentSense &cs, float &ia, float &ib, int n) {
  double sa = 0, sb = 0;
  for (int i = 0; i < n; i++) {
    PhaseCurrent_s c = cs.getPhaseCurrents();
    sa += c.a; sb += c.b;
    delayMicroseconds(200);
  }
  ia = (float)(sa / n); ib = (float)(sb / n);
}

void do_current_selftest(int idx) {
  BLDCDriver3PWM &drv = (idx == 0) ? wdriver1 : wdriver2;
  InlineCurrentSense &cs = (idx == 0) ? current_sense1 : current_sense2;
  int num = idx + 1;
  const float Vt = 2.0f;   // test voltage, well under limits (~0.2-0.3A)

  Serial.printf("--- Current AUTO test M%d (Vt=%.1fV) ---\n", num, Vt);
  drv.enable();

  // 1) baseline offset, no drive
  drv.setPwm(0, 0, 0); delay(80);
  float oa = 0, ob = 0; readCurrAvg(cs, oa, ob, 50);

  // 2) drive phase A
  drv.setPwm(Vt, 0, 0); delay(150);
  float a_ia = 0, a_ib = 0; readCurrAvg(cs, a_ia, a_ib, 50);
  drv.setPwm(0, 0, 0); delay(80);

  // 3) drive phase B
  drv.setPwm(0, Vt, 0); delay(150);
  float b_ia = 0, b_ib = 0; readCurrAvg(cs, b_ia, b_ib, 50);
  drv.setPwm(0, 0, 0);
  drv.disable();

  Serial.printf("  offset  Ia=%.3f Ib=%.3f\n", oa, ob);
  Serial.printf("  driveA  Ia=%.3f Ib=%.3f\n", a_ia, a_ib);
  Serial.printf("  driveB  Ia=%.3f Ib=%.3f\n", b_ia, b_ib);

  bool offOK  = fabsf(oa) < 0.15f && fabsf(ob) < 0.15f;       // near zero at rest
  bool chA    = (a_ia - oa) > 0.05f && (a_ia - oa) < 1.5f;    // A channel reacts to A drive
  bool chB    = (b_ib - ob) > 0.05f && (b_ib - ob) < 1.5f;    // B channel reacts to B drive
  bool signA  = (a_ib - ob) < -0.02f;                         // driving A -> Ib goes negative
  bool signB  = (b_ia - oa) < -0.02f;                         // driving B -> Ia goes negative

  Serial.printf("  [%s]offset [%s]chA [%s]chB [%s]signA [%s]signB\n",
    offOK ? "OK" : "XX", chA ? "OK" : "XX", chB ? "OK" : "XX",
    signA ? "OK" : "XX", signB ? "OK" : "XX");
  bool pass = offOK && chA && chB && signA && signB;
  Serial.printf("  RESULT M%d: %s\n", num,
    pass ? "PASS" : "FAIL -- check shunt/gain/wiring (or gain sign flip)");
}

// ============================================================================
// Menu
// ============================================================================
void print_menu() {
  Serial.println("\n========= GimbalArm HW Self-Test =========");
  Serial.printf (" IMU=%s  M1(FOC)=%s  M2(FOC)=%s\n",
                 imu_ok ? "OK" : "FAIL", m1_ok ? "OK" : "FAIL", m2_ok ? "OK" : "FAIL");
  Serial.println(" 1  I2C scan        (find ICM20948 0x68)");
  Serial.println(" 2  IMU live        (accel / gyro stream)");
  Serial.println(" 3  Encoder check   (AS5048A x2 alive+angle)");
  Serial.println(" 4  Current raw     (phase currents at rest)");
  Serial.println(" 9  Current AUTO    (inject V, verify sensor; '9' then '1'/'2')");
  Serial.println(" 5  Gimbal M1 spin  (GM4108 #1 closed-loop vel)");
  Serial.println(" 6  Gimbal M2 spin  (GM4108 #2 closed-loop vel)");
  Serial.println(" 7  AK40 status     (CAN id1,2 theta/omega)");
  Serial.println(" 8  AK40 PID hold   ('8' then '1'/'2' to pick id)");
  Serial.println(" x  STOP -> menu    0/? this menu");
  Serial.println("==========================================");
}

void do_i2c_scan() {
  Serial.println("--- I2C scan ---");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices) -- check SDA18/SCL19, pullups, power");
  else Serial.printf("  total: %d (expect 0x68 = ICM20948)\n", count);
  mode = MENU; print_menu();
}

void enter_mode(Mode m) {
  // leave any motion safely
  wmotor1.disable(); wmotor2.disable();
  for (int i = 0; i < AK40_NUM; i++) { ak[i].enabled = false; motorSetCurrent(ak[i].can_id, 0.0f); }
  mode = m;

  switch (m) {
    case I2C_SCAN: do_i2c_scan(); break;
    case IMU_LIVE:  Serial.println("--- IMU live (x=stop) ---"); break;
    case ENC_CHECK:
      Serial.println("--- Encoder check (x=stop) ---");
      Serial.printf("  ENC1(CS9) %s   ENC2(CS10) %s\n",
        encoder_alive(ENC1_CS) ? "ALIVE" : "NO RESP",
        encoder_alive(ENC2_CS) ? "ALIVE" : "NO RESP");
      break;
    case CURR_CHECK:
      Serial.println("--- Current raw at rest (x=stop) ---");
      Serial.println("  expect both phases ~0 A. large/stuck value = sensor or wiring fault");
      break;
    case CURR_SELFTEST:
      Serial.println("--- Current AUTO test: press '1'/'2' to inject & verify, x=menu ---");
      break;
    case GIMBAL_M1:
      if (!m1_ok) { Serial.println("M1 FOC not OK -- can't spin. back to menu"); mode = MENU; print_menu(); break; }
      Serial.printf("--- Gimbal M1 spin %.1f rad/s (x=stop) ---\n", SPIN_VEL_RADS);
      wmotor1.enable(); wmotor1.target = SPIN_VEL_RADS;
      break;
    case GIMBAL_M2:
      if (!m2_ok) { Serial.println("M2 FOC not OK -- can't spin. back to menu"); mode = MENU; print_menu(); break; }
      Serial.printf("--- Gimbal M2 spin %.1f rad/s (x=stop) ---\n", SPIN_VEL_RADS);
      wmotor2.enable(); wmotor2.target = SPIN_VEL_RADS;
      break;
    case AK40_STATUS:
      Serial.println("--- AK40 status stream (x=stop) ---");
      Serial.println("  no status = no CAN reply. check CTX22/CRX23, 120ohm term, motor power");
      break;
    case AK40_HOLD:
      Serial.println("--- AK40 PID hold: press '1' or '2' to pick id, x=stop ---");
      break;
    default: break;
  }
}

void handle_menu_key(char c) {
  if (c == '0' || c == '?') { mode = MENU; print_menu(); return; }
  if (c == 'x' || c == 'X') {
    Serial.println(">> stop");
    enter_mode(MENU); print_menu(); return;
  }

  // Current auto-test sub-selection (one-shot, then back to menu)
  if (mode == CURR_SELFTEST && (c == '1' || c == '2')) {
    do_current_selftest(c == '2' ? 1 : 0);
    enter_mode(MENU); print_menu();
    return;
  }

  // AK40 hold sub-selection
  if (mode == AK40_HOLD && (c == '1' || c == '2')) {
    ak_sel = (c == '2') ? 1 : 0;
    AK40Motor &m = ak[ak_sel];
    if (!m.haveStatus) { Serial.printf("AK%u: no status yet, can't hold\n", m.can_id); return; }
    m.theta_d = m.theta; m.omega_f = 0; m.enabled = true;
    Serial.printf(">> AK%u HOLD at %.1f deg (K=%.2f D=%.3f)\n",
                  m.can_id, m.theta_d / DEG2RAD, m.K, m.D);
    return;
  }

  switch (c) {
    case '1': enter_mode(I2C_SCAN); break;
    case '2': enter_mode(IMU_LIVE); break;
    case '3': enter_mode(ENC_CHECK); break;
    case '4': enter_mode(CURR_CHECK); break;
    case '9': enter_mode(CURR_SELFTEST); break;
    case '5': enter_mode(GIMBAL_M1); break;
    case '6': enter_mode(GIMBAL_M2); break;
    case '7': enter_mode(AK40_STATUS); break;
    case '8': enter_mode(AK40_HOLD); break;
    default: break;
  }
}

void process_serial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r' || ch == ' ') {
      if (serial_buf.length()) { handle_menu_key(serial_buf[0]); serial_buf = ""; }
    } else {
      handle_menu_key(ch);   // single-key, act immediately
    }
  }
}

// ============================================================================
// setup / loop
// ============================================================================
uint32_t lastAkCtrl = 0;
const uint32_t AK40_CTRL_US = 1000000UL / AK40_CTRL_HZ;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  Serial.println("\n### GimbalArm HW Self-Test booting ###");

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  Serial.print("IMU init... ");
  for (int i = 0; i < 10; i++) { if (setup_icm20948()) { imu_ok = true; break; } delay(200); }
  Serial.println(imu_ok ? "OK" : "FAILED (continue; use test 1 to scan)");

  Serial.println("Gimbal motors init (will twitch during FOC align)...");
  setup_gimbal_motors();

  ak[0] = AK40Motor{}; ak[0].can_id = M1_CAN_ID; ak[0].Kt = M1_KT; ak[0].dir = 1.0f;
  ak[0].K = K_IMP; ak[0].D = D_IMP;
  ak[1] = AK40Motor{}; ak[1].can_id = M2_CAN_ID; ak[1].Kt = M2_KT; ak[1].dir = 1.0f;
  ak[1].K = K_IMP; ak[1].D = D_IMP;

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.setFIFOFilter(ACCEPT_ALL);
  Serial.println("CAN1 init OK (1Mbps)");

  lastAkCtrl = micros();
  digitalWrite(LED_BUILTIN, HIGH);
  print_menu();
}

void loop() {
  // CAN RX always serviced so AK40 status stays fresh
  CAN_message_t rx;
  while (Can1.read(rx)) parseStatus(rx);

  // Gimbal FOC must run fast whenever a motor is enabled
  encoder1.update();
  if (m2_ok) encoder2.update();
  if (wmotor1.enabled) { wmotor1.loopFOC(); wmotor1.move(); }
  if (wmotor2.enabled) { wmotor2.loopFOC(); wmotor2.move(); }

  // AK40 200Hz hold control
  uint32_t now_us = micros();
  if ((uint32_t)(now_us - lastAkCtrl) >= AK40_CTRL_US) {
    float dt = (now_us - lastAkCtrl) * 1e-6f;
    lastAkCtrl = now_us;
    if (mode == AK40_HOLD)
      for (int i = 0; i < AK40_NUM; i++) ak_holdStep(ak[i], dt);
  }

  process_serial();

  // 5Hz live reporting per active mode
  if (millis() - last_print_ms >= 200) {
    last_print_ms = millis();
    switch (mode) {
      case IMU_LIVE:
        if (myICM.dataReady()) {
          myICM.getAGMT();
          Serial.printf("  acc[mg] %.0f %.0f %.0f | gyr[dps] %.1f %.1f %.1f\n",
            myICM.accX(), myICM.accY(), myICM.accZ(),
            myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
        } else Serial.println("  IMU no data");
        break;
      case ENC_CHECK:
        Serial.printf("  enc1=%.3f rad  enc2=%.3f rad\n",
          encoder1.getAngle(), encoder2.getAngle());
        break;
      case CURR_CHECK: {
        PhaseCurrent_s c1 = current_sense1.getPhaseCurrents();
        PhaseCurrent_s c2 = current_sense2.getPhaseCurrents();
        Serial.printf("  M1 Ia=%.3f Ib=%.3f | M2 Ia=%.3f Ib=%.3f [A]\n",
          c1.a, c1.b, c2.a, c2.b);
        break;
      }
      case GIMBAL_M1:
        Serial.printf("  M1 vel=%.2f rad/s (tgt %.1f) ang=%.2f\n",
          wmotor1.shaft_velocity, SPIN_VEL_RADS, encoder1.getAngle());
        break;
      case GIMBAL_M2:
        Serial.printf("  M2 vel=%.2f rad/s (tgt %.1f) ang=%.2f\n",
          wmotor2.shaft_velocity, SPIN_VEL_RADS, encoder2.getAngle());
        break;
      case AK40_STATUS:
        for (int i = 0; i < AK40_NUM; i++)
          Serial.printf("  AK%u %s th=%.1f deg w=%.2f rad/s\n",
            ak[i].can_id, ak[i].haveStatus ? "OK " : "---",
            ak[i].theta / DEG2RAD, ak[i].omega);
        break;
      case AK40_HOLD:
        for (int i = 0; i < AK40_NUM; i++)
          if (ak[i].enabled)
            Serial.printf("  AK%u hold th=%.1f tgt=%.1f I=%.2f A\n",
              ak[i].can_id, ak[i].theta / DEG2RAD, ak[i].theta_d / DEG2RAD, ak[i].dbg_I);
        break;
      default: break;
    }
  }
}
