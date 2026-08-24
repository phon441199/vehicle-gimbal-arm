/**
 * gimbal_arm_full.ino  —  Teensy 4.1
 * Merge of:
 *   - AK40_Impedance_Multi (CAN AK40 shoulder/elbow impedance + DOB)
 *   - gimbal_level_hold    (wrist GM4108 self-leveling via IMU z-axis)
 *
 * Pins (no conflicts):
 *   CAN1:  CTX=22, CRX=23   (AK40 shoulder=id1, elbow=id2)
 *   Wrist M1: PWM 0/1/2, EN 3, ENC1 CS=9, CS1 A0/A1
 *   Wrist M2: PWM 4/5/6, EN 7, ENC2 CS=10, CS2 A2/A3  (reserved; init may fail)
 *   SPI:   MOSI=11 MISO=12 SCK=13
 *   IMU:   I2C0  SDA=18 SCL=19
 *
 * Serial @115200 — first-char case routes:
 *   UPPERCASE → wrist commands (C/S/P/D/M/Z/?)
 *   lowercase → AK40 commands  (m1/m2/ma e d s o z t k b j g c)
 *
 *  WRIST:
 *    C           capture current phi_z as target + hold (motor1 enable)
 *    S           stop wrist hold + disable motor1
 *    P <val>     set Kp
 *    D <val>     set Kd
 *    M           toggle 10Hz monitor
 *    Z           print phi_z / accel / enc1 once
 *    ?           help
 *
 *  AK40:
 *    m1 / m2 / ma  select target motor (ma = both)
 *    e             enable impedance (DOB off)
 *    d             enable impedance + DOB
 *    s             STOP / disable
 *    o             toggle DOB on already-enabled motor
 *    z             set origin (disable first)
 *    t<deg>        target angle
 *    k<v>          K (impedance stiffness)
 *    b<v>          D (impedance damping)
 *    j<v>          DOB J_nom
 *    g<v>          DOB BW [Hz]
 *    c<amp>        manual current pulse (disable first)
 */

#include <FlexCAN_T4.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM_20948.h>

// ============================================================================
// AK40 PARAMS
// ============================================================================
#define M1_CAN_ID 1
#define M1_KT     0.0705f
#define M1_J_NOM  0.0015f
#define M1_DOB_BW 80.0f
#define M1_DIR    +1.0f
#define M2_CAN_ID 2
#define M2_KT     0.0908f
#define M2_J_NOM  0.0015f
#define M2_DOB_BW 80.0f
#define M2_DIR    +1.0f

#define GEAR_RATIO 10.0f
#define GEAR_EFF   0.90f
#define POLE_PAIRS 14
#define POS_AT_JOINT 1

#define K_IMP_INIT  2.0f
#define D_IMP_INIT  0.05f
#define OMEGA_LPF_HZ 25.0f
#define I_MAX        2.0f
#define POS_ERR_LIMIT_DEG 60.0f
#define CTRL_HZ      200

#define STATUS_PKT 0x29
#define DEG2RAD   0.0174532925f
#define RPM2RADS  0.1047197551f
#define AK40_NUM  2

typedef enum {
  CAN_PACKET_SET_DUTY        = 0,
  CAN_PACKET_SET_CURRENT     = 1,
  CAN_PACKET_SET_ORIGIN_HERE = 5,
} CAN_PACKET_ID;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

struct AK40Motor {
  uint8_t can_id;
  float Kt, J_nom, dob_bw, dir;
  float theta, omega;
  bool  haveStatus;
  float theta_d, omega_d;
  float K, D;
  bool  enabled, dob_on;
  float omega_f, dob_w, dob_v, tau_applied;
  float dbg_tau_d, dbg_dhat, dbg_I;
};
AK40Motor ak[AK40_NUM];
int activeIdx = 0;
static inline bool isTarget(int i) { return activeIdx < 0 || activeIdx == i; }

// ============================================================================
// WRIST PARAMS
// ============================================================================
#define USE_ACCEL
float Kp = 8.0f;
float Kd = 0.25f;
#define COMP_ALPHA   0.98f
#define SIGN_X   (-1.0f)
#define SIGN_Z   (+1.0f)
#define SOFT_LIMIT_RAD   1.20f

#define WM1_PWM_A   0
#define WM1_PWM_B   1
#define WM1_PWM_C   2
#define WM1_EN      3
#define WM2_PWM_A   4
#define WM2_PWM_B   5
#define WM2_PWM_C   6
#define WM2_EN      7
#define ENC1_CS     9
#define ENC2_CS     10
#define CS1_A       A0
#define CS1_B       A1
#define CS2_A       A2
#define CS2_B       A3

#define GM4108_PP          11
#define GM4108_PHASE_R     5.5f
#define SUPPLY_VOLTAGE     12.0f
#define DRIVER_VLIMIT      8.0f
#define MOTOR_VLIMIT       5.0f
#define MOTOR_ILIMIT       1.0f
#define MOTOR_VEL_LIMIT    20.0f
#define CS_SHUNT           0.01f
#define CS_GAIN            50.0f

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

float foc_zero1 = 0, foc_zero2 = 0;
bool  m1_ok = false, m2_ok = false;

// ============================================================================
// IMU
// ============================================================================
#define I2C_CLOCK_HZ   500000
#define AD0_VAL        0
#define IMU_HZ         100
#define IMU_PERIOD_US  (1000000UL / IMU_HZ)

ICM_20948_I2C myICM;

float est_phi = 0;
float gyro_bias[3] = {0, 0, 0};
float target_phi = 0;
float enc1_at_C = 0;
bool  holding = false;
unsigned long last_imu_us = 0;
unsigned long last_print_ms = 0;
bool  monitoring = false;
String serial_buf = "";

// ============================================================================
// AK40 helpers
// ============================================================================
static void buf_append_int32(uint8_t *b, int32_t v, int *i) {
  b[(*i)++] = v >> 24; b[(*i)++] = v >> 16;
  b[(*i)++] = v >> 8;  b[(*i)++] = v;
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
  uint8_t b[1] = { 0 };
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

static float dobUpdate(AK40Motor &m, float tau, float om, float dt) {
  m.dob_w += dt * m.dob_bw * (om  - m.dob_w);
  m.dob_v += dt * m.dob_bw * (tau - m.dob_v);
  return m.J_nom * m.dob_bw * (om - m.dob_w) - m.dob_v;
}

static void ak_disable(AK40Motor &m) {
  m.enabled = false;
  m.tau_applied = 0; m.dob_w = 0; m.dob_v = 0; m.omega_f = 0;
  m.dbg_tau_d = 0; m.dbg_dhat = 0; m.dbg_I = 0;
  motorSetCurrent(m.can_id, 0.0f);
}

static void ak_controlStep(AK40Motor &m, float dt) {
  if (!m.enabled || !m.haveStatus) return;
  float th = m.theta, om = m.omega;

  if (fabsf(m.theta_d - th) > POS_ERR_LIMIT_DEG * DEG2RAD) {
    ak_disable(m);
    Serial.printf("!! M%u POS LIMIT -> AUTO STOP\n", m.can_id);
    return;
  }

  if (OMEGA_LPF_HZ > 0.0f) {
    float a = 2.0f * 3.14159265f * OMEGA_LPF_HZ * dt;
    a = a / (1.0f + a);
    m.omega_f += a * (om - m.omega_f);
  } else m.omega_f = om;

  float tau_d = m.K * (m.theta_d - th) + m.D * (m.omega_d - m.omega_f);
  float d_hat = m.dob_on ? dobUpdate(m, m.tau_applied, m.omega_f, dt) : 0.0f;
  float tau   = tau_d - d_hat;

  float kt_tot = m.Kt * GEAR_RATIO * GEAR_EFF;
  float I = clampf(m.dir * tau / kt_tot, -I_MAX, I_MAX);
  m.tau_applied = m.dir * I * kt_tot;
  motorSetCurrent(m.can_id, I);
  m.dbg_tau_d = tau_d; m.dbg_dhat = d_hat; m.dbg_I = I;
}

static void ak_applyEnable(AK40Motor &m, bool dob) {
  m.theta_d = m.theta;
  m.dob_w = 0; m.dob_v = 0; m.tau_applied = 0; m.omega_f = 0;
  m.dob_on = dob; m.enabled = true;
  Serial.printf(">> M%u ENABLED %s, target=%.1f deg\n",
                m.can_id, dob ? "(imp+DOB)" : "(imp)", m.theta_d / DEG2RAD);
}

static void ak_init(AK40Motor &m, uint8_t id, float kt, float jn, float bw, float dir) {
  m = AK40Motor{};
  m.can_id = id; m.Kt = kt; m.J_nom = jn; m.dob_bw = bw; m.dir = dir;
  m.K = K_IMP_INIT; m.D = D_IMP_INIT;
}

void handle_ak40_line(String s) {
  if (s.length() == 0) return;
  if (s == "m1") { activeIdx = 0;  Serial.println(">> target M1");  return; }
  if (s == "m2") { activeIdx = 1;  Serial.println(">> target M2");  return; }
  if (s == "ma") { activeIdx = -1; Serial.println(">> target ALL"); return; }

  if (s == "e" || s == "d") {
    for (int i = 0; i < AK40_NUM; i++) if (isTarget(i)) ak_applyEnable(ak[i], s == "d");
    return;
  }
  if (s == "s") {
    for (int i = 0; i < AK40_NUM; i++) if (isTarget(i)) ak_disable(ak[i]);
    Serial.println(">> AK40 STOP"); return;
  }
  if (s == "o") {
    for (int i = 0; i < AK40_NUM; i++) if (isTarget(i)) {
      ak[i].dob_on = !ak[i].dob_on; ak[i].dob_w = 0; ak[i].dob_v = 0;
      Serial.printf(">> M%u DOB %s\n", ak[i].can_id, ak[i].dob_on ? "ON" : "OFF");
    }
    return;
  }
  if (s == "z") {
    for (int i = 0; i < AK40_NUM; i++) if (isTarget(i)) {
      if (ak[i].enabled) Serial.printf("M%u: disable first\n", ak[i].can_id);
      else { motorSetOrigin(ak[i].can_id); Serial.printf(">> M%u origin set\n", ak[i].can_id); }
    }
    return;
  }

  if (s.length() < 2) return;
  char c = s.charAt(0);
  float v = s.substring(1).toFloat();
  switch (c) {
    case 't': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) ak[i].theta_d = v*DEG2RAD;
              Serial.printf(">> target %.1f deg\n", v); break;
    case 'k': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) ak[i].K = v;
              Serial.printf(">> K=%.3f\n", v); break;
    case 'b': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) ak[i].D = v;
              Serial.printf(">> D=%.3f\n", v); break;
    case 'j': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) ak[i].J_nom = v;
              Serial.printf(">> DOB J=%.5f\n", v); break;
    case 'g': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) ak[i].dob_bw = v;
              Serial.printf(">> DOB BW=%.1f\n", v); break;
    case 'c': for (int i=0;i<AK40_NUM;i++) if(isTarget(i)) {
                if (ak[i].enabled) Serial.printf("M%u: disable first\n", ak[i].can_id);
                else { motorSetCurrent(ak[i].can_id, v);
                       Serial.printf(">> M%u %.2fA pulse\n", ak[i].can_id, v); }
              } break;
    default: Serial.printf("?? unknown ak40 cmd: '%s'\n", s.c_str());
  }
}

// ============================================================================
// Wrist helpers
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
        Serial.printf("M%d: ENC(CS=%d) no response -- skip initFOC (motor inactive). check wiring\n", num, cs);
        return false;
    }
    for (int i = 0; i < 5; i++) {
        m.initFOC();
        if (m.sensor_direction != Direction::UNKNOWN && m.zero_electric_angle > -1000.0f) {
            foc_zero = m.zero_electric_angle;
            Serial.printf("M%d FOC: zero=%.4f dir=%d (ok)\n", num, foc_zero, m.sensor_direction);
            return true;
        }
        if (!encoder_alive(cs)) {
            Serial.printf("M%d: ENC dropped during align -- abort\n", num);
            break;
        }
        Serial.printf("M%d initFOC FAIL, retry %d/5\n", num, i + 1);
        delay(300);
    }
    Serial.printf("M%d FOC: GAVE UP — sensor dead\n", num);
    return false;
}

void setup_wrist_motors() {
    encoder1.init();
    encoder2.init();
    wmotor1.linkSensor(&encoder1);
    wmotor2.linkSensor(&encoder2);

    wdriver1.voltage_power_supply = SUPPLY_VOLTAGE;
    wdriver1.voltage_limit = DRIVER_VLIMIT;
    wdriver1.init();
    wdriver2.voltage_power_supply = SUPPLY_VOLTAGE;
    wdriver2.voltage_limit = DRIVER_VLIMIT;
    wdriver2.init();
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

    for (BLDCMotor *m : {&wmotor1, &wmotor2}) {
        m->voltage_limit = MOTOR_VLIMIT;
        m->current_limit = MOTOR_ILIMIT;
        m->velocity_limit = MOTOR_VEL_LIMIT;
        m->voltage_sensor_align = 5.0f;
        m->controller = MotionControlType::torque;
        m->torque_controller = TorqueControlType::voltage;
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
    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d50bw4_n68bw8; dlpcfg.g = gyr_d51bw2_n73bw3;
    myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
    myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    ICM_20948_smplrt_t srd; srd.a = 10; srd.g = 10;
    myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
    myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);
    return true;
}

static inline float accel_phi_z(float ax, float ay) { return atan2f(-ax, -ay); }

void calibrate_gyro_bias() {
    Serial.println("Gyro bias calib -- hold still (2s)...");
    double sum[3] = {0, 0, 0};
    int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            sum[0] += myICM.gyrX(); sum[1] += myICM.gyrY(); sum[2] += myICM.gyrZ();
            n++;
        }
        delay(2);
    }
    if (n > 0) for (int i = 0; i < 3; i++)
        gyro_bias[i] = (float)(sum[i] / n) * (PI / 180.0f);
    Serial.printf("Gyro bias [rad/s]: %.5f %.5f %.5f (n=%d)\n",
                  gyro_bias[0], gyro_bias[1], gyro_bias[2], n);
#ifdef USE_ACCEL
    if (myICM.dataReady()) {
        myICM.getAGMT();
        float ax = myICM.accX(), ay = myICM.accY();
        est_phi = accel_phi_z(ax, ay);
    }
#else
    est_phi = 0;
#endif
}

void update_attitude(float dt, float &gz_rads) {
    myICM.getAGMT();
    gz_rads = myICM.gyrZ() * (PI / 180.0f) - gyro_bias[2];
    est_phi += gz_rads * dt;
#ifdef USE_ACCEL
    float ax = myICM.accX(), ay = myICM.accY(), az = myICM.accZ();
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm > 700.0f && acc_norm < 1300.0f) {
        float phi_acc = accel_phi_z(ax, ay);
        est_phi = COMP_ALPHA * est_phi + (1.0f - COMP_ALPHA) * phi_acc;
    }
#endif
}

void capture_target() {
    if (!m1_ok) {
        Serial.println("ERR: wrist motor1 FOC init failed -- C rejected");
        return;
    }
    target_phi = est_phi;
    enc1_at_C = encoder1.getAngle();
    wmotor1.enable();
    holding = true;
    Serial.printf("WRIST CAPTURED phi_z=%.3f rad — HOLD ON\n", target_phi);
}

void stop_hold() {
    holding = false;
    wmotor1.target = 0;
    wmotor1.disable();
    Serial.println("WRIST HOLD OFF — motor1 disabled");
}

bool check_soft_limit() {
    if (fabsf(encoder1.getAngle() - enc1_at_C) > SOFT_LIMIT_RAD) {
        Serial.println("!! WRIST SOFT LIMIT — emergency stop");
        stop_hold();
        return true;
    }
    return false;
}

void print_help() {
    Serial.println("── Gimbal Arm Full ──");
    Serial.println(" [WRIST] C=hold S=stop P<v>=Kp D<v>=Kd M=monitor Z=once ?=help");
    Serial.println(" [AK40 ] m1/m2/ma  e/d/s/o/z  t<deg> k<v> b<v> j<v> g<v> c<A>");
    Serial.printf ("  wrist: Kp=%.2f Kd=%.2f USE_ACCEL=%s\n", Kp, Kd,
#ifdef USE_ACCEL
        "ON");
#else
        "OFF");
#endif
}

void parse_wrist_command(String cmd) {
    if (cmd.length() == 0) return;
    char c = toupper(cmd[0]);
    String arg = cmd.substring(1); arg.trim();
    switch (c) {
        case 'C': capture_target(); break;
        case 'S': stop_hold(); break;
        case 'P': if (arg.length()) Kp = arg.toFloat();
                  Serial.printf("Kp=%.3f\n", Kp); break;
        case 'D': if (arg.length()) Kd = arg.toFloat();
                  Serial.printf("Kd=%.3f\n", Kd); break;
        case 'M': monitoring = !monitoring;
                  Serial.printf("monitor %s\n", monitoring ? "ON" : "OFF"); break;
        case 'Z':
            myICM.getAGMT();
            Serial.printf("phi_z=%.3f rad | accel ax=%.0f ay=%.0f az=%.0f mg | enc1=%.3f\n",
                          est_phi, myICM.accX(), myICM.accY(), myICM.accZ(),
                          encoder1.getAngle());
            break;
        case '?': print_help(); break;
        default: Serial.println("ERR: ? for help");
    }
}

void process_serial() {
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (serial_buf.length()) {
                String s = serial_buf; s.trim();
                if (s.length() > 0) {
                    char first = s[0];
                    if (isupper(first) || first == '?') parse_wrist_command(s);
                    else handle_ak40_line(s);
                }
                serial_buf = "";
            }
        } else if (serial_buf.length() < 64) serial_buf += ch;
    }
}

// ============================================================================
// setup / loop
// ============================================================================
uint32_t lastCtrl = 0;
const uint32_t CTRL_US = 1000000UL / CTRL_HZ;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    Serial.println("================================");
    Serial.println(" Gimbal Arm Full (AK40 x2 + Wrist self-level)");
    Serial.println("================================");

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(100);

    Serial.print("IMU init... ");
    bool imu_ok = false;
    for (int i = 0; i < 10; i++) { if (setup_icm20948()) { imu_ok = true; break; } delay(200); }
    Serial.println(imu_ok ? "OK" : "FAILED");
    if (!imu_ok) { while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(120); } }

    Serial.print("Wrist motors init... ");
    setup_wrist_motors();
    Serial.println("done");

    ak_init(ak[0], M1_CAN_ID, M1_KT, M1_J_NOM, M1_DOB_BW, M1_DIR);
    ak_init(ak[1], M2_CAN_ID, M2_KT, M2_J_NOM, M2_DOB_BW, M2_DIR);

    Can1.begin();
    Can1.setBaudRate(1000000);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.setFIFOFilter(ACCEPT_ALL);
    Serial.println("CAN1 init OK (1Mbps, AK40 ids 1,2)");

    calibrate_gyro_bias();
    lastCtrl = micros();
    last_imu_us = micros();

    print_help();
    Serial.println("WRIST: align by hand then 'C'. AK40: follow 'm1'/'z'/'e' sequence carefully.");
    Serial.println("================================");
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // --- Wrist FOC always-on ---
    encoder1.update();
    if (m2_ok) encoder2.update();
    if (wmotor1.enabled) { wmotor1.loopFOC(); wmotor1.move(); }
    if (wmotor2.enabled) { wmotor2.loopFOC(); wmotor2.move(); }

    // --- AK40 CAN RX ---
    CAN_message_t rx;
    while (Can1.read(rx)) parseStatus(rx);

    // --- AK40 200Hz control ---
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - lastCtrl) >= CTRL_US) {
        float dt = (now_us - lastCtrl) * 1e-6f;
        lastCtrl = now_us;
        for (int i = 0; i < AK40_NUM; i++) ak_controlStep(ak[i], dt);
    }

    // --- Serial ---
    process_serial();

    // --- 100Hz IMU + wrist hold ---
    now_us = micros();
    if (now_us - last_imu_us >= IMU_PERIOD_US && myICM.dataReady()) {
        float dt = (now_us - last_imu_us) * 1e-6f;
        last_imu_us = now_us;
        float gz_rads = 0;
        update_attitude(dt, gz_rads);
        if (holding) {
            if (!check_soft_limit()) {
                float e = target_phi - est_phi;
                float v1 = SIGN_X * (Kp * e - Kd * gz_rads);
                wmotor1.target = constrain(v1, -MOTOR_VLIMIT, MOTOR_VLIMIT);
            }
        }
    }

    // --- 10Hz print ---
    if (millis() - last_print_ms >= 100) {
        last_print_ms = millis();
        if (monitoring) {
            Serial.printf("[W] phi=%.1f tgt=%.1f v1=%.2f hold=%d\n",
                est_phi * 180.0f / PI,
                target_phi * 180.0f / PI,
                wmotor1.target, holding);
        }
        for (int i = 0; i < AK40_NUM; i++) {
            AK40Motor &m = ak[i];
            bool active = (fabsf(m.omega) > 0.3f) || (fabsf(m.dbg_I) > 0.05f);
            if (m.haveStatus && active) {
                Serial.printf("[AK%u] th=%.1f w=%.2f | tau_d=%.3f dhat=%.3f I=%.2f %s\n",
                              m.can_id, m.theta / DEG2RAD, m.omega,
                              m.dbg_tau_d, m.dbg_dhat, m.dbg_I,
                              !m.enabled ? "off" : (m.dob_on ? "ON+DOB" : "ON"));
            }
        }
    }
}
