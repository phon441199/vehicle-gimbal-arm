/**
 * gimbal_level_hold.ino
 * Teensy 4.1 — 1축 짐벌 자세유지 (Self-leveling), ROS2 없이 단독 동작
 *
 * 현재 구성: motor1 만 장착. motor1 회전축 ∥ IMU z축.
 *   → IMU z축 주변 tilt 만 제어 (phi_z). 다른 축은 제어 안 함.
 *   motor2 는 추후 IMU x축 회전 제어용 예약 (현재 미장착, init 실패 시 무동작).
 *
 * IMU 장착 (실측): -y 가 위. 수평에서 ay ≈ -1g, ax ≈ az ≈ 0.
 *
 * 동작:
 *   1) 부팅 → IMU/모터 init, 자이로 bias 측정(정지 유지), 모터 OFF
 *   2) 손으로 원하는 자세로 맞춤
 *   3) 'C' 입력 → 그 순간 phi_z 를 목표로 캡처 + hold 시작
 *   4) PD 피드백으로 수렴
 *
 * 자세 추정 (z축 주변 tilt):
 *   phi_z_acc = atan2(-ax, -ay)   (실측 -y up, 수평=0)
 *   gyro rate: phi_z <- gz
 *   USE_ACCEL OFF → 자이로 적분만 (단기 OK, 장기 드리프트)
 *   USE_ACCEL ON  → 상보필터 (자이로 + 가속도 중력보정)
 *   ※ encoder 는 제어용 아님 — soft-limit 안전정지 + 모니터링용 (motor1만)
 *
 * 제어: motor1 단일 PD (소각도 가정)
 *   v1 = SIGN_X * ( Kp*(target_phi - phi_z) - Kd*gz ).  DOB 없음.
 *   (변수명 SIGN_X 유지 — 추후 motor2 추가 시 SIGN_Z 활용 예정)
 *
 * 하드웨어: GM4108H ×1 (motor1), MacroBase Dual FOC 3.2 Plus, AS5048A ×1 (SPI), ICM20948 (I2C)
 * 라이브러리: SimpleFOC, SparkFun ICM20948
 */

#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM_20948.h>

// ═══════════════════════════════════════════════════════════════════════
//  튜닝 / 설정
// ═══════════════════════════════════════════════════════════════════════
#define USE_ACCEL                 // 주석 처리하면 자이로 적분만

float Kp = 8.0f;                  // [V/rad] — 'P <val>'
float Kd = 0.25f;                 // [V/(rad/s)] — 'D <val>'
#define COMP_ALPHA   0.98f        // 상보필터: 자이로 비중 (1=가속도무시)

// 부호 — 발산하면 SIGN_X 뒤집기
#define SIGN_X   (-1.0f)          // motor1 (현재: phi_z 제어) — 실측 -1
#define SIGN_Z   (+1.0f)          // motor2 예약 (미사용)

#define SOFT_LIMIT_RAD   1.20f    // C 시점 enc1 기준 ±이만큼 벗어나면 비상정지(~40deg)

// ═══════════════════════════════════════════════════════════════════════
//  핀 정의 (imu_test_firmware 와 동일)
// ═══════════════════════════════════════════════════════════════════════
#define M1_PWM_A   0
#define M1_PWM_B   1
#define M1_PWM_C   2
#define M1_EN      3
#define M2_PWM_A   4
#define M2_PWM_B   5
#define M2_PWM_C   6
#define M2_EN      7
#define ENC1_CS    9
#define ENC2_CS    10
#define CS1_A      A0
#define CS1_B      A1
#define CS2_A      A2
#define CS2_B      A3

// ═══════════════════════════════════════════════════════════════════════
//  모터 파라미터
// ═══════════════════════════════════════════════════════════════════════
#define GM4108_PP          11
#define GM4108_PHASE_R     5.5f
#define SUPPLY_VOLTAGE     12.0f
#define DRIVER_VLIMIT      8.0f
#define MOTOR_VLIMIT       5.0f
#define MOTOR_ILIMIT       1.0f
#define MOTOR_VEL_LIMIT    20.0f
#define CS_SHUNT           0.01f
#define CS_GAIN            50.0f

// ═══════════════════════════════════════════════════════════════════════
//  IMU 설정
// ═══════════════════════════════════════════════════════════════════════
#define I2C_CLOCK_HZ   500000
#define AD0_VAL        0
#define IMU_HZ         100
#define IMU_PERIOD_US  (1000000UL / IMU_HZ)

// ═══════════════════════════════════════════════════════════════════════
//  객체
// ═══════════════════════════════════════════════════════════════════════
BLDCMotor motor1 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCMotor motor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

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

ICM_20948_I2C myICM;

// ═══════════════════════════════════════════════════════════════════════
//  상태 변수  (phi_z = IMU z축 주변 tilt)
// ═══════════════════════════════════════════════════════════════════════
float est_phi = 0;                      // 추정 각 [rad] (z축 주변 tilt)
float gyro_bias[3] = {0, 0, 0};         // [rad/s]  (0=x,1=y,2=z)
float target_phi = 0;                   // C 시점 캡처
float enc1_at_C = 0;                    // soft-limit 기준 (motor1만)
bool  holding = false;
unsigned long last_imu_us = 0;
unsigned long last_print_ms = 0;
bool  monitoring = false;
String serial_buf = "";

// AS5048 생존 체크 — ANGLE 레지스터 응답이 0xFFFF(=MISO 플로팅=무응답)가 아니면 살아있음.
// error flag(bit14)는 SimpleFOC 읽기로 latch될 수 있어 판정에서 제외 (데이터는 유효).
bool encoder_alive(int cs) {
    for (int t = 0; t < 4; t++) {
        SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
        digitalWrite(cs, LOW);
        uint16_t raw = SPI.transfer16(0xFFFF);   // ANGLE 읽기 명령
        digitalWrite(cs, HIGH);
        SPI.endTransaction();
        if (raw != 0xFFFF) return true;
        delayMicroseconds(100);
    }
    return false;
}

// 엔코더 살아있을 때만 initFOC(=정렬, 모터 움직임). 죽었으면 스킵 → 무동작.
bool init_one_motor(BLDCMotor &m, int cs, float &foc_zero, int num) {
    m.init();
    if (!encoder_alive(cs)) {
        Serial.printf("M%d: ENC(CS=%d) 무응답 — initFOC 스킵 (모터 무동작). 배선 확인\n", num, cs);
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
            Serial.printf("M%d: ENC dropped during align — 중단\n", num);
            break;
        }
        Serial.printf("M%d initFOC FAIL, retry %d/5\n", num, i + 1);
        delay(300);
    }
    Serial.printf("M%d FOC: GAVE UP — sensor dead\n", num);
    return false;
}

void setup_motors() {
    encoder1.init();
    encoder2.init();
    motor1.linkSensor(&encoder1);
    motor2.linkSensor(&encoder2);

    driver1.voltage_power_supply = SUPPLY_VOLTAGE;
    driver1.voltage_limit = DRIVER_VLIMIT;
    driver1.init();
    driver2.voltage_power_supply = SUPPLY_VOLTAGE;
    driver2.voltage_limit = DRIVER_VLIMIT;
    driver2.init();
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    current_sense1.linkDriver(&driver1);
    current_sense1.init();
    current_sense1.gain_a *= -1; current_sense1.gain_b *= -1;
    current_sense1.skip_align = true;
    current_sense2.linkDriver(&driver2);
    current_sense2.init();
    current_sense2.gain_a *= -1; current_sense2.gain_b *= -1;
    current_sense2.skip_align = true;
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

// ═══════════════════════════════════════════════════════════════════════
//  IMU 초기화 (mag 미사용)
// ═══════════════════════════════════════════════════════════════════════
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

// phi_z 계산: IMU z축 주변 tilt. 실측 -y up (ay≈-1g 수평).
// 수평=0. 부호: gyro 적분 부호와 일치하도록 atan2(-ax, -ay) 사용.
//   level: ax=0, ay=-1g → atan2(0, +1g) = 0 ✓
//   +30° around z: ax=-0.5g, ay=-0.866g → atan2(+0.5, +0.866) = +30° ✓
// 발산 시 SIGN_X 토글.
static inline float accel_phi_z(float ax, float ay) { return atan2f(-ax, -ay); }

void calibrate_gyro_bias() {
    Serial.println("Gyro bias calib — 정지 유지 (2s)...");
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

// ═══════════════════════════════════════════════════════════════════════
//  자세 추정 (z축 주변 tilt 만)
// ═══════════════════════════════════════════════════════════════════════
void update_attitude(float dt, float &gz_rads) {
    myICM.getAGMT();
    gz_rads = myICM.gyrZ() * (PI / 180.0f) - gyro_bias[2];

    est_phi += gz_rads * dt;

#ifdef USE_ACCEL
    float ax = myICM.accX(), ay = myICM.accY(), az = myICM.accZ();
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);   // mg
    if (acc_norm > 700.0f && acc_norm < 1300.0f) {   // 1g 근처에서만 중력보정
        float phi_acc = accel_phi_z(ax, ay);
        est_phi = COMP_ALPHA * est_phi + (1.0f - COMP_ALPHA) * phi_acc;
    }
#endif
}

// ═══════════════════════════════════════════════════════════════════════
//  제어
// ═══════════════════════════════════════════════════════════════════════
void capture_target() {
    if (!m1_ok) {
        Serial.println("ERR: motor1 FOC init 실패 — C 거부. 엔코더 배선 확인 후 재부팅");
        return;
    }
    target_phi = est_phi;
    enc1_at_C = encoder1.getAngle();
    motor1.enable();
    holding = true;
    Serial.printf("CAPTURED target phi_z=%.3f rad — HOLD ON\n", target_phi);
}

void stop_hold() {
    holding = false;
    motor1.target = 0;
    motor1.disable();
    Serial.println("HOLD OFF — motor1 disabled");
}

bool check_soft_limit() {
    if (fabsf(encoder1.getAngle() - enc1_at_C) > SOFT_LIMIT_RAD) {
        Serial.println("!! SOFT LIMIT — emergency stop");
        stop_hold();
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
//  Serial 명령
// ═══════════════════════════════════════════════════════════════════════
void print_help() {
    Serial.println("── Gimbal Level Hold (z-axis only, motor1) ──");
    Serial.println("  C          현재 phi_z를 목표로 캡처 + hold 시작");
    Serial.println("  S          정지 (모터 off)");
    Serial.println("  P <val>    Kp 설정");
    Serial.println("  D <val>    Kd 설정");
    Serial.println("  M          모니터링 토글 (10Hz)");
    Serial.println("  Z          현재 추정자세/엔코더 1회 출력");
    Serial.println("  ?          도움말");
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
            if (serial_buf.length()) { parse_command(serial_buf); serial_buf = ""; }
        } else serial_buf += ch;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("================================");
    Serial.println(" Gimbal Self-Leveling 1-axis (motor1 / IMU z)");
    Serial.println(" IMU mount: -y up (ay~-1g at level)");
    Serial.println("================================");

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(100);

    Serial.print("IMU init... ");
    bool imu_ok = false;
    for (int i = 0; i < 10; i++) { if (setup_icm20948()) { imu_ok = true; break; } delay(200); }
    Serial.println(imu_ok ? "OK" : "FAILED");
    if (!imu_ok) { while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(120); } }

    Serial.print("Motors init... ");
    setup_motors();
    Serial.println("OK");

    calibrate_gyro_bias();

    last_imu_us = micros();
    Serial.println("손으로 자세 맞춘 뒤 'C' 입력. '?' 도움말.");
    Serial.println("================================");
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // 엔코더 멀티턴 추적 유지 — 모터 off 상태(C 전 수동 정렬)에서도 매 루프 샘플링.
    encoder1.update();
    if (m2_ok) encoder2.update();   // motor2 살아있을 때만

    // FOC 항상 고속 구동
    if (motor1.enabled) { motor1.loopFOC(); motor1.move(); }
    if (motor2.enabled) { motor2.loopFOC(); motor2.move(); }   // 미장착 시 enabled=false 유지

    process_serial();

    unsigned long now_us = micros();
    if (now_us - last_imu_us < IMU_PERIOD_US) return;
    if (!myICM.dataReady()) return;
    float dt = (now_us - last_imu_us) * 1e-6f;
    last_imu_us = now_us;

    float gz_rads = 0;
    update_attitude(dt, gz_rads);

    if (holding) {
        if (!check_soft_limit()) {
            float e = target_phi - est_phi;
            float v1 = SIGN_X * (Kp * e - Kd * gz_rads);   // motor1 -> phi_z
            motor1.target = constrain(v1, -MOTOR_VLIMIT, MOTOR_VLIMIT);
        }
    }

    if (monitoring && millis() - last_print_ms >= 100) {
        last_print_ms = millis();
        Serial.printf("phi=%.1f deg | tgt=%.1f | v1=%.2f | hold=%d\n",
            est_phi * 180.0f / PI,
            target_phi * 180.0f / PI,
            motor1.target, holding);
    }
}
