 /**
 * gimbal_level_hold_esp32.ino  -  LOLIN D32 (ESP32)
 * 1-axis gimbal self-leveling (ESP32 port of Teensy gimbal_level_hold).
 *
 * motor1 only: rotation axis || IMU z-axis -> controls tilt around z (phi_z).
 * motor2 reserved (IMU x-axis), no-op if its encoder is absent.
 *
 * IMU mount (measured): -y up. At level ay ~= -1g, ax ~= az ~= 0.
 *
 * Flow:
 *   1) boot -> IMU/motor init, gyro bias calib (keep still), motors OFF
 *   2) hand-pose to desired attitude
 *   3) 'C' -> capture phi_z target + start hold
 *   4) PD feedback converges
 *
 * Attitude (tilt around z):
 *   phi_z_acc = atan2(-ax, -ay)   (-y up, level=0)
 *   USE_ACCEL off -> gyro integration only; on -> complementary filter
 *   encoder is NOT for control -- soft-limit safety stop + monitor (motor1)
 *
 * Control: motor1 single PD (small angle)
 *   v1 = SIGN_X * (Kp*(target - phi_z) - Kd*gz).  No DOB.
 *
 * HW: GM4108H x1 (motor1), MKS DUAL FOC, AS5048A (SPI), ICM20948 (SPI, shared bus)
 *
 * ---- ESP32 pins (shared SPI bus, verified) ---------------------------------
 *   motor1 PWM 25/26/27 EN 14   motor2 PWM 16/17/4 EN 12
 *   SPI SCK18 MISO19 MOSI23   ENC1_CS21 ENC2_CS22   IMU_CS13
 *   current M0 36/39  M1 34/32
 * ----------------------------------------------------------------------------
 * Libraries: SimpleFOC, SparkFun ICM_20948
 */

#include <SimpleFOC.h>
#include <SPI.h>
#include <ICM_20948.h>

// ---- tuning ----------------------------------------------------------------
#define USE_ACCEL                 // comment out for gyro-integration only

float Kp = 8.0f;                  // [V/rad]  'P <val>'
float Kd = 0.25f;                 // [V/(rad/s)] 'D <val>'
#define COMP_ALPHA   0.993f       // complementary gyro weight. =tau/(tau+dt), tau~0.49s @300Hz
                                  // (was 0.98 @100Hz; raise with IMU_HZ to keep accel-correction tau)

#define SIGN_X   (-1.0f)          // motor1 (phi_z). flip if it diverges
#define SIGN_Z   (+1.0f)          // motor2 reserved (unused)

// Safety limits. PRIMARY = attitude error (control-divergence guard, frame-correct:
// independent of base motion -- the gimbal may swing the encoder far while keeping
// the platform level, esp. with QDD base motion). SECONDARY = wide encoder guard
// for mechanical/cable wind-up only.
#define ATT_LIMIT_RAD    0.61f    // |est_phi - target_phi| e-stop (~35deg) = control failed
#define ENC_LIMIT_RAD    2.60f    // |enc1 - enc1_at_C| e-stop (~150deg) = mechanical/cable guard

// ---- ESO (Extended State Observer = Level3 DOB, PDF section 3) --------------
//   plant: w_dot = b0*i + f,  b0 = Kt/J,  f = d/J (total disturbance accel).
//   states [x1=theta, x2=omega, x3=f]; measure theta(encoder), input i(current).
//     x1' = x2        + l1*(theta-x1)
//     x2' = b0*i + x3 + l2*(theta-x1)
//     x3' =             l3*(theta-x1)
//   error char poly = s^3 + l1 s^2 + l2 s + l3. Triple pole at -wo (Gao):
//     l1 = 3*wo,  l2 = 3*wo^2,  l3 = wo^3.   d_hat = J*x3.  ONE knob wo.
//   Runs in the FAST loop (kHz) so wo*dt stays small -> high bandwidth, stable.
#define J_MEAS       6.8e-5f      // measured motor1 inertia [kg·m^2] (2026-06-13)
#define DOB_R        5.5f         // phase R [ohm] for torque->voltage map
#define DOB_KT       0.154f       // motor1 Kt [N·m/A]
#define DOB_VCLAMP   3.0f         // |comp voltage| safety clamp [V] (raised: 1V was throttling ESO)
float   dob_scale = 0.7f;         // comp authority (signed). POSITIVE = correct sign for ESO
                                  // (verified by graph 2026-06-14: A-0.9 diverges, A+0.9 clean)  'A <val>'
bool    dob_on = false;           // compensation enable  'B' (ESO always runs)
float   eso_x1 = 0, eso_x2 = 0, eso_x3 = 0, d_hat = 0;   // ESO states; d_hat = J*x3
float   eso_wo = 2.0f * PI * 15.0f;          // observer bandwidth [rad/s]  'G <hz>'
float   eso_l1 = 0, eso_l2 = 0, eso_l3 = 0;  // gains, set from wo
const float eso_b0 = DOB_KT / J_MEAS;        // input gain Kt/J (~2265)
float   v_pd = 0;                            // PD output (100Hz), applied+ESO-comp in fast loop
bool    telem_on = false;                    // CSV telemetry stream for PC bridge  'T'
unsigned long last_telem_ms = 0;
// fast-loop current LPF: Iq sampled every main-loop iter (~kHz) and low-passed.
#define IQ_TAU       0.008f           // ~20Hz current filter
float   iq_filt = 0;
unsigned long last_iq_us = 0;

void set_eso_bw(float wo) {           // place ESO triple pole at -wo
  eso_wo = wo;
  eso_l1 = 3.0f * wo;
  eso_l2 = 3.0f * wo * wo;
  eso_l3 = wo * wo * wo;
}

// ---- pins (ESP32) ----------------------------------------------------------
#define M1_PWM_A 25
#define M1_PWM_B 26
#define M1_PWM_C 27
#define M1_EN    14
#define M2_PWM_A 16
#define M2_PWM_B 17
#define M2_PWM_C 4
#define M2_EN    12

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define ENC1_CS  21
#define ENC2_CS  22
#define IMU_CS   13

#define CS1_A 36
#define CS1_B 39
#define CS2_A 34
#define CS2_B 32

// ---- motor params ----------------------------------------------------------
#define GM4108_PP        11
#define GM4108_PHASE_R   5.5f
#define SUPPLY_VOLTAGE   12.0f
#define DRIVER_VLIMIT    8.0f
#define MOTOR_VLIMIT     5.0f
#define MOTOR_ILIMIT     1.0f
#define MOTOR_VEL_LIMIT  20.0f
#define CS_SHUNT         0.01f
#define CS_GAIN          50.0f
#define ENC_SPI_HZ       1000000UL
#define IMU_SPI_HZ       1000000UL   // 1MHz: margin for 90cm shared bus

// ---- IMU ------------------------------------------------------------------
#define IMU_HZ        300         // attitude loop rate (mag unused here; ODR set to 375 below)
#define IMU_PERIOD_US (1000000UL / IMU_HZ)

// ---- inertia (J) measurement: J = Kt * Iq / alpha (PDF section 3) ----------
// Current(torque)-controlled, NOT voltage: holding Iq constant makes torque
// constant regardless of back-EMF -> clean constant alpha. Magnitude control
// (fixed + drive direction) so it can't run away on a current-sign mismatch.
#define J_TEST_KT     0.154f      // motor1 Kt [N·m/A] (project_motor_test_progress)
#define J_TEST_R      5.5f        // phase R [ohm] for Vq feedforward
#define J_TEST_IQ     0.12f       // default |Iq| level 1 [A] (level 2 = 2x); net torque must beat friction
#define J_TEST_MS     40          // short window: bounds travel + keeps Vq < limit
#define J_TEST_DT_US  500         // 0.5ms sampling
#define J_TEST_N      (J_TEST_MS * 1000 / J_TEST_DT_US)
#define J_CUR_KI      1500.0f     // current-loop integral gain [V/(A·s)]

// ---- objects ---------------------------------------------------------------
BLDCMotor motor1 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCMotor motor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

MagneticSensorSPIConfig_s AS5048_SPI_SLOW = {
  .spi_mode = SPI_MODE1, .clock_speed = ENC_SPI_HZ, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC2_CS);

InlineCurrentSense current_sense1 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS1_A, CS1_B);
InlineCurrentSense current_sense2 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS2_A, CS2_B);

float foc_zero1 = 0, foc_zero2 = 0;
bool  m1_ok = false, m2_ok = false;
bool  imu_ok = false;

ICM_20948_SPI myICM;

// ---- state (phi_z = tilt around IMU z) -------------------------------------
float est_phi = 0;
float gyro_bias[3] = {0, 0, 0};
float target_phi = 0;
float enc1_at_C = 0;
bool  holding = false;
unsigned long last_imu_us = 0;
unsigned long last_print_ms = 0;
bool  monitoring = false;
String serial_buf = "";

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

bool init_one_motor(BLDCMotor &m, int cs, float &foc_zero, int num) {
  m.init();
  if (!encoder_alive(cs)) {
    Serial.printf("M%d: ENC(CS=%d) no response -- skip initFOC (no-op). check wiring\n", num, cs);
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

void setup_motors() {
  encoder1.init(&SPI);
  encoder2.init(&SPI);
  motor1.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);

  driver1.voltage_power_supply = SUPPLY_VOLTAGE; driver1.voltage_limit = DRIVER_VLIMIT; driver1.init();
  driver2.voltage_power_supply = SUPPLY_VOLTAGE; driver2.voltage_limit = DRIVER_VLIMIT; driver2.init();
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  current_sense1.linkDriver(&driver1); current_sense1.init();
  current_sense1.gain_a *= -1; current_sense1.gain_b *= -1; current_sense1.skip_align = true;
  current_sense2.linkDriver(&driver2); current_sense2.init();
  current_sense2.gain_a *= -1; current_sense2.gain_b *= -1; current_sense2.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);
  motor2.linkCurrentSense(&current_sense2);

  for (BLDCMotor *m : {&motor1, &motor2}) {
    m->voltage_limit = MOTOR_VLIMIT;
    m->current_limit = MOTOR_ILIMIT;
    m->velocity_limit = MOTOR_VEL_LIMIT;
    m->voltage_sensor_align = 5.0f;
    m->controller = MotionControlType::torque;
    m->torque_controller = TorqueControlType::voltage;
  }

  m1_ok = init_one_motor(motor1, ENC1_CS, foc_zero1, 1);
  m2_ok = init_one_motor(motor2, ENC2_CS, foc_zero2, 2);
  motor1.disable();
  motor2.disable();
}

bool setup_icm20948() {
  myICM.begin(IMU_CS, SPI, IMU_SPI_HZ);
  if (myICM.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fss; fss.a = gpm4; fss.g = dps500;
  myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_dlpcfg_t dlpcfg; dlpcfg.a = acc_d50bw4_n68bw8; dlpcfg.g = gyr_d51bw2_n73bw3;
  myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
  myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  ICM_20948_smplrt_t srd; srd.a = 2; srd.g = 2;   // ODR=1125/(1+2)=375Hz (>300Hz loop)
  myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
  myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);
  return (myICM.status == ICM_20948_Stat_Ok);
}

static inline float accel_phi_z(float ax, float ay) { return atan2f(-ax, -ay); }

void calibrate_gyro_bias() {
  Serial.println("Gyro bias calib -- keep still (2s)...");
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
  if (n > 0) for (int i = 0; i < 3; i++) gyro_bias[i] = (float)(sum[i] / n) * (PI / 180.0f);
  Serial.printf("Gyro bias [rad/s]: %.5f %.5f %.5f (n=%d)\n",
                gyro_bias[0], gyro_bias[1], gyro_bias[2], n);
#ifdef USE_ACCEL
  if (myICM.dataReady()) {
    myICM.getAGMT();
    est_phi = accel_phi_z(myICM.accX(), myICM.accY());
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
  if (!imu_ok) {
    Serial.println("ERR: IMU not ok -- 'C' refused. fix IMU first");
    return;
  }
  if (!m1_ok) {
    Serial.println("ERR: motor1 FOC init failed -- 'C' refused. check encoder, reboot");
    return;
  }
  target_phi = est_phi;
  enc1_at_C = encoder1.getAngle();
  // fresh ESO each hold: seed position state to current encoder angle, rest 0
  eso_x1 = encoder1.getAngle(); eso_x2 = 0; eso_x3 = 0; d_hat = 0;
  iq_filt = 0; v_pd = 0; last_iq_us = micros();
  motor1.enable();
  holding = true;
  Serial.printf("CAPTURED target phi_z=%.3f rad -- HOLD ON (ESO comp=%s)\n",
                target_phi, dob_on ? "ON" : "OFF");
}

void stop_hold() {
  holding = false;
  v_pd = 0;
  motor1.target = 0;
  motor1.disable();
  Serial.println("HOLD OFF -- motor1 disabled");
}

bool check_soft_limit() {
  if (fabsf(est_phi - target_phi) > ATT_LIMIT_RAD) {   // primary: attitude (control failed)
    Serial.println("!! ATTITUDE LIMIT -- control diverged, e-stop");
    stop_hold();
    return true;
  }
  if (fabsf(encoder1.getAngle() - enc1_at_C) > ENC_LIMIT_RAD) {  // mechanical/cable guard
    Serial.println("!! ENCODER LIMIT -- mechanical travel, e-stop");
    stop_hold();
    return true;
  }
  return false;
}

// quadratic least-squares theta = c0 + c1 t + c2 t^2 over [i0,i1) -> alpha = 2 c2
static float quad_alpha(const float *t_s, const float *th, int i0, int i1) {
  double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0, Y0 = 0, Y1 = 0, Y2 = 0;
  double t0 = t_s[i0];
  for (int i = i0; i < i1; i++) {
    double t = t_s[i] - t0, y = th[i], t2 = t * t;   // shift t so fit is well-conditioned
    S0 += 1; S1 += t; S2 += t2; S3 += t2 * t; S4 += t2 * t2;
    Y0 += y; Y1 += y * t; Y2 += y * t2;
  }
  double A[3][3] = {{S0, S1, S2}, {S1, S2, S3}, {S2, S3, S4}};
  double b[3] = {Y0, Y1, Y2};
  for (int k = 0; k < 3; k++) {                 // Gauss-Jordan -> diagonal
    int p = k;
    for (int r = k + 1; r < 3; r++) if (fabs(A[r][k]) > fabs(A[p][k])) p = r;
    for (int cc = 0; cc < 3; cc++) { double tmp = A[k][cc]; A[k][cc] = A[p][cc]; A[p][cc] = tmp; }
    { double tmp = b[k]; b[k] = b[p]; b[p] = tmp; }
    for (int r = 0; r < 3; r++) if (r != k) {
      double f = A[r][k] / A[k][k];
      for (int cc = 0; cc < 3; cc++) A[r][cc] -= f * A[k][cc];
      b[r] -= f * b[k];
    }
  }
  return (float)(2.0 * b[2] / A[2][2]);
}

// One constant-torque pulse from the CURRENT (rest) position: hold |Iq|=target
// via integral current loop (magnitude, +drive), short window. quad fit -> alpha.
static void rest_pulse(float Iq_target, float &alpha, float &iq_mean, float &travel) {
  static float t_s[J_TEST_N], th[J_TEST_N];
  double iqsum = 0;
  motor1.controller = MotionControlType::torque;
  motor1.voltage_limit = MOTOR_VLIMIT;
  motor1.enable();
  float Vq = J_TEST_R * Iq_target;
  unsigned long t0 = micros(), next = t0, prev = t0;
  int n = 0;
  while (n < J_TEST_N) {
    unsigned long now = micros();
    encoder1.update();
    motor1.loopFOC();
    float iqa = fabsf(current_sense1.getFOCCurrents(motor1.electrical_angle).q);
    float dt  = (now - prev) * 1e-6f; prev = now;
    Vq += J_CUR_KI * (Iq_target - iqa) * dt;
    Vq  = constrain(Vq, 0.0f, (float)MOTOR_VLIMIT);
    motor1.move(Vq);
    if ((long)(now - next) >= 0) {
      t_s[n] = (now - t0) * 1e-6f; th[n] = encoder1.getAngle(); iqsum += iqa;
      n++; next += J_TEST_DT_US;
    }
  }
  motor1.move(0);
  alpha   = quad_alpha(t_s, th, 0, J_TEST_N);
  iq_mean = iqsum / J_TEST_N;
  travel  = th[J_TEST_N - 1] - th[0];
}

// PD return to th_home (s = encoder sign for +Vq, so Vq = s*(Kp*e - Kd*w) drives
// theta toward home regardless of motor polarity). Brakes + re-centers.
static void home_to(float th_home, float s, int ms) {
  unsigned long t = millis();
  while (millis() - t < (unsigned long)ms) {
    encoder1.update(); motor1.loopFOC();
    float e = th_home - encoder1.getAngle();
    float w = encoder1.getVelocity();
    float Vq = s * (3.0f * e - 0.10f * w);
    motor1.move(constrain(Vq, -(float)MOTOR_VLIMIT, (float)MOTOR_VLIMIT));
  }
}

// Inertia ID on motor1. Two constant-torque pulses (Iq1, Iq2) BOTH from rest at
// the SAME home position: matching velocity profile (cancels viscous) and
// matching position (cancels gravity/cable spring). J = Kt*(Iq2-Iq1)/(a2-a1).
void measure_inertia(float Iq1) {
  if (!m1_ok) { Serial.println("ERR: motor1 FOC not ok -- J test refused"); return; }
  float Iq2 = Iq1 * 2.0f;
  Serial.printf("--- J test motor1 (rest+rehome 2-level): Iq1=%.3f Iq2=%.3f A, %dms ---\n",
                Iq1, Iq2, J_TEST_MS);
  holding = false;

  float th_home = encoder1.getAngle();
  float a1, iq1, tr1, a2, iq2, tr2;

  rest_pulse(Iq1, a1, iq1, tr1);
  float s = (tr1 >= 0) ? 1.0f : -1.0f;           // encoder sign for +Vq, from pulse1 travel
  home_to(th_home, s, 1000);

  rest_pulse(Iq2, a2, iq2, tr2);
  home_to(th_home, s, 1000);
  motor1.move(0); motor1.disable();

  Serial.printf("  L1: alpha=%.1f Iq=%.3f travel=%.3f | L2: alpha=%.1f Iq=%.3f travel=%.3f (s=%+.0f)\n",
                a1, iq1, tr1, a2, iq2, tr2, s);
  float dAlpha = a2 - a1;
  if (fabsf(dAlpha) < 2.0f) {
    Serial.println("  !! alpha2~alpha1: raise Iq1 (J <Iq1>) so both pulses move clearly.");
    return;
  }
  float J = fabsf(J_TEST_KT * (iq2 - iq1) / dAlpha);
  Serial.printf("  >> J(motor1) = Kt*(Iq2-Iq1)/(a2-a1) = %.3e kg·m^2\n", J);
  Serial.println("  repeat 2-3x: should be stable now (same start pos + both from rest).");
}

void print_help() {
  Serial.println("-- Gimbal Level Hold (z-axis only, motor1) --");
  Serial.println("  C        capture current phi_z as target + start hold");
  Serial.println("  S        stop (motor off)");
  Serial.println("  P <val>  set Kp");
  Serial.println("  D <val>  set Kd");
  Serial.println("  M        toggle monitor (10Hz human)");
  Serial.println("  T        toggle CSV telemetry (50Hz, for PC ROS2 bridge)");
  Serial.println("  J [Iq1]  measure motor1 inertia (2-level current ctrl, Iq2=2*Iq1)");
  Serial.println("  B        toggle ESO disturbance compensation");
  Serial.println("  G <hz>   set ESO bandwidth wo (higher=faster reject, more noise)");
  Serial.println("  A <val>  set comp authority (negative flips sign)");
  Serial.println("  Z        print attitude/encoder once");
  Serial.println("  ?        help");
  Serial.printf ("  (Kp=%.2f Kd=%.2f USE_ACCEL=%s)\n", Kp, Kd,
#ifdef USE_ACCEL
    "ON");
#else
    "OFF");
#endif
}

void parse_command(String &cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);
  String arg = cmd.substring(1); arg.trim();
  switch (c) {
    case 'C': capture_target(); break;
    case 'S': stop_hold(); break;
    case 'P': if (arg.length()) Kp = arg.toFloat(); Serial.printf("Kp=%.3f\n", Kp); break;
    case 'D': if (arg.length()) Kd = arg.toFloat(); Serial.printf("Kd=%.3f\n", Kd); break;
    case 'M': monitoring = !monitoring; Serial.printf("monitor %s\n", monitoring ? "ON" : "OFF"); break;
    case 'T': telem_on = !telem_on; Serial.printf("telemetry %s\n", telem_on ? "ON" : "OFF"); break;
    case 'J': { float iq = arg.length() ? arg.toFloat() : J_TEST_IQ;
                measure_inertia(constrain(iq, 0.02f, 0.5f)); break; }
    case 'B': dob_on = !dob_on;
              Serial.printf("ESO compensation %s\n", dob_on ? "ON" : "OFF"); break;
    case 'G': if (arg.length()) set_eso_bw(2.0f * PI * arg.toFloat());
              Serial.printf("ESO bandwidth wo = %.1f Hz\n", eso_wo / (2.0f * PI)); break;
    case 'A': if (arg.length()) dob_scale = arg.toFloat();
              Serial.printf("comp authority = %.2f (neg = flip sign)\n", dob_scale); break;
    case 'Z':
      myICM.getAGMT();
      Serial.printf("phi_z=%.3f rad | accel ax=%.0f ay=%.0f az=%.0f mg | enc1=%.3f\n",
                    est_phi, myICM.accX(), myICM.accY(), myICM.accZ(), encoder1.getAngle());
      break;
    case '?': print_help(); break;
    default: Serial.println("ERR: ? for help");
  }
}

void process_serial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serial_buf.length()) { parse_command(serial_buf); serial_buf = ""; }
    } else serial_buf += ch;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("================================");
  Serial.println(" Gimbal Self-Leveling 1-axis (motor1 / IMU z) [ESP32]");
  Serial.println(" IMU mount: -y up (ay~-1g at level)");
  Serial.println("================================");

  // shared SPI bus: bring up once, park all CS high
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
  pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);
  pinMode(IMU_CS,  OUTPUT); digitalWrite(IMU_CS,  HIGH);

  // encoders first -- they warm the shared bus before the IMU's first transfer
  Serial.print("Motors init... ");
  setup_motors();
  Serial.println("OK");

  set_eso_bw(eso_wo);   // compute ESO gains l1,l2,l3 from default bandwidth

  Serial.print("IMU init... ");
  for (int i = 0; i < 10; i++) { if (setup_icm20948()) { imu_ok = true; break; } delay(200); }
  Serial.println(imu_ok ? "OK" : "FAILED (check IMU_CS=13, AD0=MISO, power; 'Z' to retry attitude)");

  if (imu_ok) calibrate_gyro_bias();
  else Serial.println("WARN: no IMU -- 'C' hold disabled until IMU ok");

  last_imu_us = micros();
  Serial.println("Hand-pose then press 'C'. '?' for help.");
  Serial.println("================================");
}

void loop() {
  encoder1.update();
  if (m2_ok) encoder2.update();

  if (motor1.enabled) { motor1.loopFOC(); motor1.move(); }
  if (motor2.enabled) { motor2.loopFOC(); motor2.move(); }

  // ===== FAST loop (~kHz): current filter + ESO + apply motor command =====
  if (motor1.enabled) {
    unsigned long nu = micros();
    float dtq = (nu - last_iq_us) * 1e-6f; last_iq_us = nu;
    if (dtq > 0.0f && dtq < 0.05f) {
      // current filter (electrical_angle valid right after loopFOC above)
      float iqr = current_sense1.getFOCCurrents(motor1.electrical_angle).q;
      iq_filt += (dtq / (IQ_TAU + dtq)) * (iqr - iq_filt);

      // ESO update: states [theta, omega, f=d/J] from encoder theta + current
      float th  = encoder1.getAngle();
      float err = th - eso_x1;
      eso_x1 += dtq * (eso_x2 + eso_l1 * err);
      eso_x2 += dtq * (eso_b0 * iq_filt + eso_x3 + eso_l2 * err);
      eso_x3 += dtq * (eso_l3 * err);
      d_hat = J_MEAS * eso_x3;                          // disturbance torque estimate

      // apply held PD (100Hz) + ESO disturbance compensation
      float v = v_pd;
      if (dob_on) {
        float comp = -dob_scale * (DOB_R / DOB_KT) * d_hat;
        v += constrain(comp, -DOB_VCLAMP, DOB_VCLAMP);
      }
      motor1.target = constrain(v, -MOTOR_VLIMIT, MOTOR_VLIMIT);
    }
  }

  process_serial();

  unsigned long now_us = micros();
  if (now_us - last_imu_us < IMU_PERIOD_US) return;
  if (!myICM.dataReady()) return;
  float dt = (now_us - last_imu_us) * 1e-6f;
  last_imu_us = now_us;

  float gz_rads = 0;
  update_attitude(dt, gz_rads);

  // ===== outer attitude loop (IMU rate): PD only -> v_pd (fast loop applies it) =====
  if (holding) {
    if (!check_soft_limit()) {
      float e = target_phi - est_phi;
      v_pd = SIGN_X * (Kp * e - Kd * gz_rads);
    }
  }

  if (monitoring && millis() - last_print_ms >= 100) {
    last_print_ms = millis();
    Serial.printf("phi=%.1f tgt=%.1f | v=%.2f vpd=%.2f | w_eso=%.2f d_hat=%.4f | wo=%.0fHz A=%.2f dob=%d\n",
      est_phi * 180.0f / PI, target_phi * 180.0f / PI, motor1.target, v_pd,
      eso_x2, d_hat, eso_wo / (2.0f * PI), dob_scale, dob_on);
  }

  // CSV telemetry: #T,ms,phi,tgt,err,v,vpd,d_hat,w_eso,dob (deg/V/Nm). comp = v-vpd
  if (telem_on && millis() - last_telem_ms >= 20) {
    last_telem_ms = millis();
    Serial.printf("#T,%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.4f,%.2f,%d\n",
      millis(), est_phi * 180.0f / PI, target_phi * 180.0f / PI,
      (est_phi - target_phi) * 180.0f / PI, motor1.target, v_pd, d_hat, eso_x2, dob_on ? 1 : 0);
  }
}
