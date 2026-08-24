/**
 * imu_test_firmware.ino
 * Teensy 4.1 — ICM20948 IMU + GM4108 듀얼 BLDC 모터 테스트
 *
 * 모드 전환: 아래 #define 주석 토글 후 리플래시
 *   MOTOR_TEST_MODE — Serial 명령으로 모터 테스트 + Kt 측정
 *   ROS_MODE        — micro-ROS IMU 발행 + 모터 ROS 토픽
 *
 * 하드웨어:
 *   모터: GM4108H ×2 (11 pole pairs)
 *   드라이버: MacroBase Dual FOC 3.2 Plus (3PWM + enable)
 *   엔코더: AS5048A ×2 (SPI, CS=9/10)
 *   전류센서: 인라인 2상 (shunt 1mΩ, gain 50, ±33A)
 *   IMU: ICM20948 (I2C, SDA=18, SCL=19)
 *
 * 필요 라이브러리:
 *   - SimpleFOC (Library Manager)
 *   - SparkFun 9DoF IMU Breakout - ICM 20948
 *   - Madgwick by Arduino
 *   - micro_ros_arduino (ROS_MODE만)
 */

// ═══════════════════════════════════════════════════════════════════════
//  모드 선택 — 하나만 활성화
// ═══════════════════════════════════════════════════════════════════════
#define MOTOR_TEST_MODE
// #define ROS_MODE

// ═══════════════════════════════════════════════════════════════════════
//  공통 헤더
// ═══════════════════════════════════════════════════════════════════════
#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>

#ifdef ROS_MODE
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#endif

// ═══════════════════════════════════════════════════════════════════════
//  핀 정의
// ═══════════════════════════════════════════════════════════════════════
// Motor CH1 (GM4108 #1)
#define M1_PWM_A   0
#define M1_PWM_B   1
#define M1_PWM_C   2
#define M1_EN      3

// Motor CH2 (GM4108 #2)
#define M2_PWM_A   4
#define M2_PWM_B   5
#define M2_PWM_C   6
#define M2_EN      7

// Encoders — AS5048A (SPI0 공유)
#define ENC1_CS    9
#define ENC2_CS    10

// Current Sensors — inline 2-phase
#define CS1_A      A0    // Teensy header pin 14
#define CS1_B      A1    // Teensy header pin 15
#define CS2_A      A2    // Teensy header pin 16
#define CS2_B      A3    // Teensy header pin 17

// ═══════════════════════════════════════════════════════════════════════
//  모터 파라미터
// ═══════════════════════════════════════════════════════════════════════
#define GM4108_PP          11      // pole pairs (22 poles / 2)
#define GM4108_PHASE_R     5.5f    // phase resistance [Ω] (실측: 두 상간 ~11Ω ÷ 2, 접촉저항 포함 가능)
#define GM4108_KT          0.0f    // [N·m/A] — K1/K2 명령으로 측정 후 입력
#define SUPPLY_VOLTAGE     12.0f
#define DRIVER_VLIMIT      6.0f
#define MOTOR_VLIMIT       3.0f
#define MOTOR_ILIMIT       1.0f
#define MOTOR_VEL_LIMIT    20.0f   // rad/s

// 전류센서 (MacroBase Dual FOC 3.2 Plus)
#define CS_SHUNT           0.01f   // 10mΩ (R010, MKS 예제 코드 확인)
#define CS_GAIN            50.0f   // ±3.3A → ±1.65V

// ═══════════════════════════════════════════════════════════════════════
//  SimpleFOC 객체
// ═══════════════════════════════════════════════════════════════════════
BLDCMotor motor1 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCMotor motor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

MagneticSensorSPIConfig_s AS5048_SPI_SLOW = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 100000,    // 100kHz (30cm 케이블 대응)
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_SPI_SLOW, ENC2_CS);

InlineCurrentSense current_sense1 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS1_A, CS1_B);
InlineCurrentSense current_sense2 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CS2_A, CS2_B);

float foc_zero1 = 0;
float foc_zero2 = 0;

// ═══════════════════════════════════════════════════════════════════════
//  IMU 객체
// ═══════════════════════════════════════════════════════════════════════
#define I2C_CLOCK_HZ   100000
#define AD0_VAL        0
#define IMU_HZ         50
#define IMU_PERIOD_MS  (1000 / IMU_HZ)

ICM_20948_I2C myICM;
Madgwick filter;

#define MAG_CAL_DURATION_MS 15000
float mag_offset[3] = {0, 0, 0};
float mag_scale[3]  = {1, 1, 1};

// ═══════════════════════════════════════════════════════════════════════
//  FOC 방향 자동 체크 — initFOC 후 π offset 필요 여부 판별
//  +V → +ω 가 되도록, 두 옵션 중 더 큰 (부호 포함) 속도를 만드는 쪽 선택
// ═══════════════════════════════════════════════════════════════════════
void check_foc_direction(BLDCMotor &motor, float &foc_zero, int motor_num) {
    float orig = foc_zero;
    motor.enable();
    motor.target = 1.5f;

    unsigned long tc = millis();
    while (millis() - tc < 800) { motor.loopFOC(); motor.move(); }
    float spd0 = motor.shaft_velocity;

    motor.zero_electric_angle = orig + _PI;
    tc = millis();
    while (millis() - tc < 800) { motor.loopFOC(); motor.move(); }
    float spdpi = motor.shaft_velocity;

    // 부호 포함 비교: 더 양수(=정방향)인 쪽 선택
    if (spd0 >= spdpi) {
        motor.zero_electric_angle = orig;
    } else {
        foc_zero = orig + _PI;
        if (foc_zero > _2PI) foc_zero -= _2PI;
        motor.zero_electric_angle = foc_zero;
    }

    motor.target = 0;
    motor.disable();
    Serial.printf("M%d dir check: spd0=%.2f spdPI=%.2f -> zero=%.4f\n",
                  motor_num, spd0, spdpi, foc_zero);
}

// ═══════════════════════════════════════════════════════════════════════
//  모터 초기화
// ═══════════════════════════════════════════════════════════════════════
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
    current_sense1.gain_a *= -1;
    current_sense1.gain_b *= -1;
    current_sense1.skip_align = true;

    current_sense2.linkDriver(&driver2);
    current_sense2.init();
    current_sense2.gain_a *= -1;
    current_sense2.gain_b *= -1;
    current_sense2.skip_align = true;

    motor1.linkCurrentSense(&current_sense1);
    motor2.linkCurrentSense(&current_sense2);

    motor1.voltage_limit = MOTOR_VLIMIT;
    motor1.current_limit = MOTOR_ILIMIT;
    motor1.velocity_limit = MOTOR_VEL_LIMIT;
    motor1.voltage_sensor_align = 3.0f;
    motor1.controller = MotionControlType::torque;
    motor1.torque_controller = TorqueControlType::voltage;

    motor2.voltage_limit = MOTOR_VLIMIT;
    motor2.current_limit = MOTOR_ILIMIT;
    motor2.velocity_limit = MOTOR_VEL_LIMIT;
    motor2.voltage_sensor_align = 3.0f;
    motor2.controller = MotionControlType::torque;
    motor2.torque_controller = TorqueControlType::voltage;

    motor1.init();
    motor1.initFOC();
    foc_zero1 = motor1.zero_electric_angle;
    Serial.printf("M1 FOC: zero_angle=%.4f dir=%d\n", foc_zero1, motor1.sensor_direction);
    check_foc_direction(motor1, foc_zero1, 1);

    motor2.init();
    motor2.initFOC();
    foc_zero2 = motor2.zero_electric_angle;
    Serial.printf("M2 FOC: zero_angle=%.4f dir=%d\n", foc_zero2, motor2.sensor_direction);
    check_foc_direction(motor2, foc_zero2, 2);
}

// ═══════════════════════════════════════════════════════════════════════
//  IMU 초기화
// ═══════════════════════════════════════════════════════════════════════
bool setup_icm20948() {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) return false;

    ICM_20948_fss_t fss;
    fss.a = gpm4;
    fss.g = dps500;
    myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d50bw4_n68bw8;
    dlpcfg.g = gyr_d51bw2_n73bw3;
    myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
    myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    myICM.enableDLPF(ICM_20948_Internal_Gyr, true);

    ICM_20948_smplrt_t srd;
    srd.a = 10;
    srd.g = 10;
    myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
    myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);

    myICM.startupMagnetometer();
    return true;
}

void calibrate_magnetometer() {
    float mn[3] = { 1e9,  1e9,  1e9};
    float mx[3] = {-1e9, -1e9, -1e9};
    unsigned long start = millis();
    while (millis() - start < MAG_CAL_DURATION_MS) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float m[3] = {myICM.magX(), myICM.magY(), myICM.magZ()};
            for (int i = 0; i < 3; i++) {
                if (m[i] < mn[i]) mn[i] = m[i];
                if (m[i] > mx[i]) mx[i] = m[i];
            }
        }
        digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
        delay(10);
    }
    for (int i = 0; i < 3; i++) mag_offset[i] = (mx[i] + mn[i]) / 2.0f;
    float delta[3], avg = 0;
    for (int i = 0; i < 3; i++) { delta[i] = (mx[i] - mn[i]) / 2.0f; avg += delta[i]; }
    avg /= 3.0f;
    for (int i = 0; i < 3; i++) mag_scale[i] = (delta[i] > 0.01f) ? avg / delta[i] : 1.0f;
}

void eulerToQuaternion(float roll, float pitch, float yaw,
                       float &qw, float &qx, float &qy, float &qz) {
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
}

// ═══════════════════════════════════════════════════════════════════════
// ╔═══════════════════════════════════════════════════════════════════╗
// ║                      MOTOR_TEST_MODE                            ║
// ╚═══════════════════════════════════════════════════════════════════╝
// ═══════════════════════════════════════════════════════════════════════
#ifdef MOTOR_TEST_MODE

String serial_buf = "";
bool monitoring_m1 = false;
bool monitoring_m2 = false;
bool imu_monitoring = false;
bool encoder_watching = false;
bool hold_m1 = false;
bool hold_m2 = false;
float hold_target1 = 0;
float hold_target2 = 0;
float hold_kp = 3.0f;
float hold_kd = 0.3f;
unsigned long last_monitor_ms = 0;
unsigned long last_imu_ms = 0;

// ── 센서 읽기 ────────────────────────────────────────────────────────
void print_sensor_readings() {
    encoder1.update();
    encoder2.update();
    PhaseCurrent_s c1 = current_sense1.getPhaseCurrents();
    PhaseCurrent_s c2 = current_sense2.getPhaseCurrents();

    Serial.println("── Sensor Readings ──");
    Serial.printf("ENC1: angle=%.4f rad  vel=%.4f rad/s\n",
                  encoder1.getAngle(), encoder1.getVelocity());
    Serial.printf("ENC2: angle=%.4f rad  vel=%.4f rad/s\n",
                  encoder2.getAngle(), encoder2.getVelocity());
    Serial.printf("CS1:  Ia=%.4f A  Ib=%.4f A\n", c1.a, c1.b);
    Serial.printf("CS2:  Ia=%.4f A  Ib=%.4f A\n", c2.a, c2.b);
    Serial.printf("RAW ADC: A0=%d  A1=%d  A2=%d  A3=%d\n",
                  analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3));

    if (motor1.enabled) {
        Serial.printf("M1 FOC: Iq=%.4f A  Id=%.4f A  vel=%.4f rad/s\n",
                      motor1.current.q, motor1.current.d, motor1.shaft_velocity);
    }
    if (motor2.enabled) {
        Serial.printf("M2 FOC: Iq=%.4f A  Id=%.4f A  vel=%.4f rad/s\n",
                      motor2.current.q, motor2.current.d, motor2.shaft_velocity);
    }

    if (myICM.dataReady()) {
        myICM.getAGMT();
        Serial.printf("IMU: ax=%.2f ay=%.2f az=%.2f mg  gx=%.2f gy=%.2f gz=%.2f dps\n",
                      myICM.accX(), myICM.accY(), myICM.accZ(),
                      myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
    }
}

// ── Kt 측정 루틴 (연속 ramp + 선형 회귀) ─────────────────────────────
//   0V → Vmax 까지 한 번에 천천히 ramp하면서 (vq, iq, ω) 연속 샘플링.
//   정전압 구간이 없어 slip-stick 한계주기를 회피, 다회전으로 코깅 평균.
//   모델: V = R·Iq + Ke·ω
//     2-param fit:    R 과 Ke 동시 추정 (R도 검증)
//     R-fixed fit:    motor.phase_resistance 사용
//   둘이 비슷하면 측정값 신뢰 가능, 다르면 R 추정치 또는 비선형성 의심.
#define KT_DEFAULT_VMAX       3.0f
#define KT_DEFAULT_RAMP_MS    60000UL    // 0→Vmax 총 ramp 시간
#define KT_DEFAULT_PRINT_HZ   20         // CSV 출력 레이트
#define KT_SAMPLE_PERIOD_US   5000UL     // 200Hz 회귀 누적
#define KT_MIN_OMEGA          1.0f       // 이 미만은 수집 제외 (정지/잡음)
#define KT_MIN_SAMPLES        100

void run_kt_test(BLDCMotor &motor, MagneticSensorSPI &enc,
                 InlineCurrentSense &cs, String &cmd) {
    float v_max = KT_DEFAULT_VMAX;
    unsigned long total_ramp_ms = KT_DEFAULT_RAMP_MS;
    int print_hz = KT_DEFAULT_PRINT_HZ;

    // 파라미터 파싱: K1 <Vmax> <total_ramp_ms> <print_hz>
    {
        String params = cmd.substring(2);
        params.trim();
        String tok[3];
        int n = 0;
        while (params.length() > 0 && n < 3) {
            int sp = params.indexOf(' ');
            if (sp < 0) { tok[n++] = params; break; }
            tok[n++] = params.substring(0, sp);
            params = params.substring(sp + 1);
            params.trim();
        }
        if (n >= 1) v_max         = tok[0].toFloat();
        if (n >= 2) total_ramp_ms = (unsigned long)tok[1].toInt();
        if (n >= 3) print_hz      = tok[2].toInt();
    }
    if (v_max < 0.1f) v_max = 0.1f;
    if (total_ramp_ms < 1000) total_ramp_ms = 1000;
    if (print_hz < 1)   print_hz = 1;
    if (print_hz > 200) print_hz = 200;

    Serial.println("==================================================");
    Serial.println("  Kt MEASUREMENT (Continuous Ramp + Regression)");
    Serial.printf("  Vmax=%.2fV  TotalRamp=%lums  PrintHz=%d\n",
                  v_max, total_ramp_ms, print_hz);
    Serial.printf("  Phase R(known)=%.2f Ohm  Pole Pairs=%d\n",
                  motor.phase_resistance, GM4108_PP);
    Serial.println("  Model: V = R*Iq + Ke*omega  →  2-param (R,Ke) + R-fixed fit");
    Serial.println("  S to abort");
    Serial.println("==================================================");
    Serial.println("t_ms,vq,iq,id,omega,collected");

    motor.enable();
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    // 회귀 누적 (double — 12000+ 샘플에서 precision 보장)
    double S_iqiq = 0, S_ww = 0, S_iqw = 0;
    double S_v_iq = 0, S_v_w  = 0;
    double S_iq   = 0, S_w   = 0, S_v = 0;  // 평균용
    int n_collected = 0;

    unsigned long t0 = millis();
    unsigned long last_samp_us  = micros();
    unsigned long last_print_us = micros();
    unsigned long print_period_us = 1000000UL / (unsigned long)print_hz;

    while (true) {
        unsigned long el = millis() - t0;
        if (el >= total_ramp_ms) break;

        float frac = (float)el / (float)total_ramp_ms;
        motor.target = v_max * frac;
        motor.loopFOC();
        motor.move();

        unsigned long now_us = micros();
        if (now_us - last_samp_us >= KT_SAMPLE_PERIOD_US) {
            last_samp_us = now_us;

            DQCurrent_s dq = cs.getFOCCurrents(motor.electrical_angle + _PI_2);
            float vq = motor.target;
            float iq = dq.q;
            float w  = motor.shaft_velocity;
            int collected = 0;

            if (fabsf(w) > KT_MIN_OMEGA) {
                S_iqiq += (double)iq * iq;
                S_ww   += (double)w  * w;
                S_iqw  += (double)iq * w;
                S_v_iq += (double)vq * iq;
                S_v_w  += (double)vq * w;
                S_iq   += iq;
                S_w    += w;
                S_v    += vq;
                n_collected++;
                collected = 1;
            }

            if (now_us - last_print_us >= print_period_us) {
                last_print_us = now_us;
                Serial.printf("%lu,%.3f,%.4f,%.4f,%.3f,%d\n",
                              el, vq, iq, dq.d, w, collected);
            }
        }

        if (Serial.available()) {
            char c = Serial.peek();
            if (c == 'S' || c == 's') {
                Serial.read();
                motor.target = 0; motor.disable();
                Serial.println("ABORTED");
                return;
            }
        }
    }

    // ── 종료: 0V까지 1초 ramp 다운 ──────────────────────────────────
    float v_from = motor.target;
    unsigned long dt0 = millis();
    while (millis() - dt0 < 1000) {
        float f = (float)(millis() - dt0) / 1000.0f;
        motor.target = v_from * (1.0f - f);
        motor.loopFOC();
        motor.move();
    }
    motor.target = 0;
    motor.disable();

    // ── 결과 ────────────────────────────────────────────────────────
    Serial.println("==================================================");
    Serial.printf("  Samples collected (|w|>%.2f): %d\n", KT_MIN_OMEGA, n_collected);

    if (n_collected < KT_MIN_SAMPLES) {
        Serial.println("  ERR: insufficient samples — Vmax/ramp_ms 늘리거나 모터가 도는지 확인");
        Serial.println("==================================================");
        return;
    }

    // 2-param fit: V = R*Iq + Ke*omega (no intercept)
    // Normal equation:
    //   [S_iqiq  S_iqw] [R ]   [S_v_iq]
    //   [S_iqw   S_ww ] [Ke] = [S_v_w ]
    double det = S_iqiq * S_ww - S_iqw * S_iqw;
    Serial.println("  ── 2-parameter fit (R and Ke both estimated) ──");
    if (fabs(det) < 1e-9) {
        Serial.println("  ERR: singular (Iq and omega not independent enough)");
    } else {
        double R_fit  = (S_ww * S_v_iq - S_iqw * S_v_w) / det;
        double Ke_fit = (S_iqiq * S_v_w - S_iqw * S_v_iq) / det;
        Serial.printf("    R_fit  = %.4f Ohm\n", R_fit);
        Serial.printf("    Ke_fit = %.5f V*s/rad  →  Kt = %.5f N*m/A\n", Ke_fit, Ke_fit);
    }

    // R-fixed fit: Ke = Σ((V - R·Iq)·ω) / Σω²
    double R_known = motor.phase_resistance;
    double S_resid_w = S_v_w - R_known * S_iqw;
    double Ke_R_fixed = S_resid_w / S_ww;
    Serial.println("  ── 1-parameter fit (R fixed from motor.phase_resistance) ──");
    Serial.printf("    R(fixed) = %.2f Ohm\n", R_known);
    Serial.printf("    Ke       = %.5f V*s/rad  →  Kt = %.5f N*m/A\n",
                  Ke_R_fixed, Ke_R_fixed);

    // 데이터 통계
    double v_avg = S_v / n_collected;
    double iq_avg = S_iq / n_collected;
    double w_avg = S_w / n_collected;
    Serial.printf("  Avg: V=%.3f  Iq=%.4f  omega=%.3f rad/s\n",
                  v_avg, iq_avg, w_avg);

    Serial.println("==================================================");
    Serial.println("  Interpretation:");
    Serial.println("   - 두 추정치(R_fit Ke vs R_fixed Ke)가 비슷하면 둘 다 신뢰 OK");
    Serial.println("   - R_fit 이 phase_resistance 와 다르면 온도/전류센서 영향");
    Serial.println("   - Ke 변동이 크면 비선형성 (포화/마찰) — CSV 확인");
    Serial.println("==================================================");
}

// ── L (인덕턴스) 측정 — 스텝 응답 ──────────────────────────────────
//  A-B 상에 스텝 전압 인가 → 전류 상승 곡선에서 시정수 τ 추출
//  두 상 직렬: V = 2R·I + 2L·dI/dt → τ = L/R → L = τ·R
#define L_SAMPLE_MAX    1000
#define L_DEFAULT_V     2.0f

static float  l_cur_buf[L_SAMPLE_MAX];
static uint32_t l_time_buf[L_SAMPLE_MAX];

void run_inductance_test(BLDCMotor &motor, BLDCDriver3PWM &drv,
                         InlineCurrentSense &cs, String &cmd) {
    float test_v = L_DEFAULT_V;
    String params = cmd.substring(2);
    params.trim();
    if (params.length() > 0) test_v = params.toFloat();
    if (test_v > DRIVER_VLIMIT) test_v = DRIVER_VLIMIT;

    Serial.println("==================================================");
    Serial.println("  INDUCTANCE MEASUREMENT (Step Response)");
    Serial.printf("  Vstep=%.2fV  R=%.2f Ohm\n", test_v, motor.phase_resistance);
    Serial.println("  Applying voltage across phase A-B...");
    Serial.println("==================================================");

    motor.disable();
    drv.enable();

    float center = drv.voltage_power_supply / 2.0f;

    // zero-current baseline (드라이버 ON, 전압차 0)
    drv.setPwm(center, center, center);
    delay(200);

    float i_zero = 0;
    for (int i = 0; i < 200; i++) {
        PhaseCurrent_s pc = cs.getPhaseCurrents();
        i_zero += pc.a;
        delayMicroseconds(50);
    }
    i_zero /= 200.0f;

    // 스텝 인가: A상 = center + V/2, B상 = center - V/2
    int n = 0;
    uint32_t t0 = micros();
    drv.setPwm(center + test_v / 2.0f, center - test_v / 2.0f, center);

    while (n < L_SAMPLE_MAX) {
        l_time_buf[n] = micros() - t0;
        PhaseCurrent_s pc = cs.getPhaseCurrents();
        l_cur_buf[n] = pc.a - i_zero;
        n++;
        if (l_time_buf[n - 1] > 5000) break;  // 5ms 충분
    }

    // 스텝 제거
    drv.setPwm(center, center, center);
    delay(100);
    drv.disable();

    if (n < 10) {
        Serial.println("ERR: Too few samples");
        return;
    }

    // 정상상태 전류 (마지막 20% 평균)
    int tail = n * 4 / 5;
    float i_final = 0;
    for (int i = tail; i < n; i++) i_final += l_cur_buf[i];
    i_final /= (float)(n - tail);

    float i_expected = test_v / (2.0f * motor.phase_resistance);

    // τ 찾기: |I(t)| 가 |I_final| × 63.2% 에 도달하는 시점
    float i_target = fabsf(i_final) * 0.6321f;
    float tau_us = 0;
    bool found = false;
    for (int i = 1; i < n; i++) {
        if (fabsf(l_cur_buf[i]) >= i_target) {
            float i_prev = fabsf(l_cur_buf[i - 1]);
            float i_curr = fabsf(l_cur_buf[i]);
            float frac = (i_target - i_prev) / (i_curr - i_prev + 1e-9f);
            tau_us = (float)l_time_buf[i - 1] +
                     frac * (float)(l_time_buf[i] - l_time_buf[i - 1]);
            found = true;
            break;
        }
    }

    float tau_s = tau_us * 1e-6f;
    float L_val = tau_s * motor.phase_resistance;

    // CSV 출력
    Serial.println("t(us), I(A)");
    for (int i = 0; i < n; i++) {
        Serial.printf("%lu, %.5f\n", (unsigned long)l_time_buf[i], l_cur_buf[i]);
    }

    Serial.println("==================================================");
    Serial.printf("  Samples      = %d\n", n);
    Serial.printf("  I_final(meas)= %.4f A\n", i_final);
    Serial.printf("  I_final(calc)= %.4f A  (V/2R)\n", i_expected);
    if (found) {
        Serial.printf("  tau          = %.1f us\n", tau_us);
        Serial.printf("  L = tau * R  = %.4f mH\n", L_val * 1000.0f);
        Serial.println("──────────────────────────────────────────────────");
        Serial.printf("  >> motor.phase_inductance = %.6f;\n", L_val);
    } else {
        Serial.println("  WARNING: tau not found (current rose too fast or too slow)");
        Serial.println("  Try different Vstep or check wiring");
    }
    Serial.println("==================================================");
}

// ── Rattle 진단: 느린 램프 + 연속 CSV 로깅 ─────────────────────────
//   드르륵 발생 시점의 (시간, 전압, 위치, 속도, Iq, Id)를 한 줄에 기록.
//   속도가 최근 추세 대비 30% 이상 떨어지면 event=1 로 자동 표시.
//
//   분석 가이드:
//     event=1 행들의 ang_norm_deg 가 비슷  → 위치 의존 (코깅/베어링/배선)
//     event=1 행들의 vel       이 비슷  → 기계적 공진
//     event=1 행들의 vq        이 비슷  → 전기적/제어 이슈
//     event=1 의 vq 가 매우 낮음만 발생 → 슬립-스틱/스틱션
#define RD_DEFAULT_VMAX     3.0f
#define RD_DEFAULT_RAMP_MS  30000UL
#define RD_DEFAULT_LOG_HZ   50
#define RD_WIN              20      // 추세 평균 윈도우 (RD_LOG_HZ 기준 약 400ms)
#define RD_DROP_RATIO       0.30f   // 30% 이상 하락이면 rattle 이벤트

void run_rattle_diag(BLDCMotor &motor, MagneticSensorSPI &enc,
                     InlineCurrentSense &cs, String &cmd) {
    float v_max = RD_DEFAULT_VMAX;
    unsigned long ramp_ms = RD_DEFAULT_RAMP_MS;
    int log_hz = RD_DEFAULT_LOG_HZ;

    // 파라미터 파싱: X1 <Vmax> <ramp_ms> <log_hz>
    {
        String params = cmd.substring(2);
        params.trim();
        String tok[3];
        int n = 0;
        while (params.length() > 0 && n < 3) {
            int sp = params.indexOf(' ');
            if (sp < 0) { tok[n++] = params; break; }
            tok[n++] = params.substring(0, sp);
            params = params.substring(sp + 1);
            params.trim();
        }
        if (n >= 1) v_max   = tok[0].toFloat();
        if (n >= 2) ramp_ms = (unsigned long)tok[1].toInt();
        if (n >= 3) log_hz  = tok[2].toInt();
    }
    if (v_max < 0.1f) v_max = 0.1f;
    if (ramp_ms < 500) ramp_ms = 500;
    if (log_hz < 5)   log_hz = 5;
    if (log_hz > 200) log_hz = 200;

    Serial.println("==================================================");
    Serial.println("  RATTLE DIAGNOSTIC (slow ramp + continuous CSV log)");
    Serial.printf("  Vmax=%.2fV  Ramp=%lums  LogRate=%dHz\n", v_max, ramp_ms, log_hz);
    Serial.println("  Listen for 드르륵, note timestamp, then check CSV rows nearby");
    Serial.println("==================================================");
    Serial.println("t_ms,vq,ang_deg,ang_norm_deg,rev,vel,iq,id,elec_ang,event");

    motor.enable();
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    static float v_buf[RD_WIN];
    int v_idx = 0;
    int v_n   = 0;

    unsigned long t0 = millis();
    unsigned long last_log_us = micros();
    unsigned long log_period_us = 1000000UL / (unsigned long)log_hz;
    int event_count = 0;

    while (true) {
        unsigned long el = millis() - t0;
        if (el >= ramp_ms) break;

        float frac = (float)el / (float)ramp_ms;
        motor.target = v_max * frac;
        motor.loopFOC();
        motor.move();

        unsigned long now_us = micros();
        if (now_us - last_log_us >= log_period_us) {
            last_log_us = now_us;

            float vel = motor.shaft_velocity;

            int event = 0;
            if (v_n >= RD_WIN) {
                float avg_long = 0;
                for (int k = 0; k < RD_WIN; k++) avg_long += v_buf[k];
                avg_long /= RD_WIN;
                if (avg_long > 0.5f && vel < avg_long * (1.0f - RD_DROP_RATIO)) {
                    event = 1;
                    event_count++;
                }
            }

            v_buf[v_idx] = vel;
            v_idx = (v_idx + 1) % RD_WIN;
            if (v_n < RD_WIN) v_n++;

            float mech_ang = enc.getAngle();
            int rev = (int)floorf(mech_ang / _2PI);
            float ang_norm = mech_ang - rev * _2PI;
            if (ang_norm < 0) { ang_norm += _2PI; rev -= 1; }

            DQCurrent_s dq = cs.getFOCCurrents(motor.electrical_angle + _PI_2);

            Serial.printf("%lu,%.3f,%.2f,%.2f,%d,%.3f,%.4f,%.4f,%.3f,%d\n",
                el, motor.target,
                mech_ang * 180.0f / _PI,
                ang_norm * 180.0f / _PI,
                rev, vel, dq.q, dq.d, motor.electrical_angle, event);
        }

        if (Serial.available()) {
            char c = Serial.peek();
            if (c == 'S' || c == 's') {
                Serial.read();
                motor.target = 0; motor.disable();
                Serial.println("ABORTED");
                return;
            }
        }
    }

    // 종료: 1초간 0V로 램프 다운
    float v_from = motor.target;
    unsigned long down_t0 = millis();
    while (millis() - down_t0 < 1000) {
        float f = (float)(millis() - down_t0) / 1000.0f;
        motor.target = v_from * (1.0f - f);
        motor.loopFOC();
        motor.move();
    }
    motor.target = 0;
    motor.disable();

    Serial.println("==================================================");
    Serial.printf("  Done. Rattle events flagged: %d\n", event_count);
    Serial.println("  Analyze event=1 rows:");
    Serial.println("    ang_norm_deg 비슷 → 위치 의존 (코깅/베어링/배선)");
    Serial.println("    vel          비슷 → 기계적 공진");
    Serial.println("    vq           비슷 → 전기/제어 이슈");
    Serial.println("    vq 매우 낮음만 → 슬립-스틱");
    Serial.println("==================================================");
}

// ── Zero Angle 캘리브레이션 Sweep ────────────────────────────────────
void run_zero_cal(BLDCMotor &motor, int motor_num) {
    float orig = motor.zero_electric_angle;
    float cal_v = 1.5f;

    Serial.println("==================================================");
    Serial.printf("  ZERO ANGLE CALIBRATION — Motor %d\n", motor_num);
    Serial.printf("  Voltage=%.1fV  Original zero=%.4f  dir=%d\n",
                  cal_v, orig, motor.sensor_direction);
    Serial.println("  Press S to abort");
    Serial.println("==================================================");

    motor.enable();
    motor.target = cal_v;

    unsigned long t0 = millis();
    while (millis() - t0 < 1000) { motor.loopFOC(); motor.move(); }

    const int NC = 24;
    float cs_step = _2PI / NC;
    float cs_speed[NC];
    float cs_std[NC];
    int best_ci = 0;

    Serial.println("\n[Coarse] offset(rad), speed(rad/s), std");

    for (int i = 0; i < NC; i++) {
        float off = -_PI + i * cs_step;
        motor.zero_electric_angle = orig + off;

        t0 = millis();
        while (millis() - t0 < 400) { motor.loopFOC(); motor.move(); }

        float sv = 0, sv2 = 0; int ns = 0;
        t0 = millis();
        while (millis() - t0 < 300) {
            motor.loopFOC(); motor.move();
            float v = motor.shaft_velocity;
            sv += v; sv2 += v * v; ns++;
        }
        cs_speed[i] = sv / ns;
        cs_std[i] = sqrtf(fabsf(sv2 / ns - cs_speed[i] * cs_speed[i]));

        Serial.printf("%.3f, %.4f, %.4f\n", off, cs_speed[i], cs_std[i]);

        if (fabsf(cs_speed[i]) > fabsf(cs_speed[best_ci])) best_ci = i;

        if (Serial.available() && (Serial.peek() == 'S' || Serial.peek() == 's')) {
            Serial.read();
            motor.zero_electric_angle = orig;
            motor.target = 0; motor.disable();
            Serial.println("ABORTED");
            return;
        }
    }

    float coarse_best = -_PI + best_ci * cs_step;

    const float FR = cs_step;
    const float FS = _PI / 90.0f;
    int NF = (int)(2.0f * FR / FS) + 1;
    float fine_best_off = coarse_best;
    float fine_best_spd = cs_speed[best_ci];
    float fine_best_std = cs_std[best_ci];

    Serial.printf("\n[Fine] around offset=%.3f (±%.1f deg)\n",
                  coarse_best, FR * 180.0f / _PI);
    Serial.println("offset(rad), speed(rad/s), std");

    for (int i = 0; i < NF; i++) {
        float off = coarse_best - FR + i * FS;
        motor.zero_electric_angle = orig + off;

        t0 = millis();
        while (millis() - t0 < 300) { motor.loopFOC(); motor.move(); }

        float sv = 0, sv2 = 0; int ns = 0;
        t0 = millis();
        while (millis() - t0 < 300) {
            motor.loopFOC(); motor.move();
            float v = motor.shaft_velocity;
            sv += v; sv2 += v * v; ns++;
        }
        float avg = sv / ns;
        float std_v = sqrtf(fabsf(sv2 / ns - avg * avg));

        Serial.printf("%.4f, %.4f, %.4f\n", off, avg, std_v);

        if (fabsf(avg) > fabsf(fine_best_spd)) {
            fine_best_spd = avg;
            fine_best_off = off;
            fine_best_std = std_v;
        }

        if (Serial.available() && (Serial.peek() == 'S' || Serial.peek() == 's')) {
            Serial.read();
            motor.zero_electric_angle = orig;
            motor.target = 0; motor.disable();
            Serial.println("ABORTED");
            return;
        }
    }

    motor.zero_electric_angle = orig + fine_best_off;
    motor.target = 0;
    t0 = millis();
    while (millis() - t0 < 500) { motor.loopFOC(); motor.move(); }
    motor.disable();

    if (motor_num == 1) foc_zero1 = orig + fine_best_off;
    else foc_zero2 = orig + fine_best_off;

    Serial.println("\n==================================================");
    Serial.printf("  Original zero:  %.4f\n", orig);
    Serial.printf("  Best offset:    %.4f rad (%.1f deg elec)\n",
                  fine_best_off, fine_best_off * 180.0f / _PI);
    Serial.printf("  New zero:       %.4f\n", orig + fine_best_off);
    Serial.printf("  Speed at best:  %.4f rad/s (std=%.4f)\n",
                  fine_best_spd, fine_best_std);
    Serial.println("  Applied! F1/F2 0 = no offset from new base");
    Serial.println("==================================================");
}

// ── 긴급 정지 ────────────────────────────────────────────────────────
void emergency_stop() {
    motor1.target = 0;
    motor2.target = 0;
    motor1.disable();
    motor2.disable();
    monitoring_m1 = false;
    monitoring_m2 = false;
    hold_m1 = false;
    hold_m2 = false;
    Serial.println("STOPPED: All motors disabled");
}

// ── 도움말 ───────────────────────────────────────────────────────────
void print_help() {
    Serial.println("── GM4108 Dual Motor Test Commands ──");
    Serial.println("  E1 / E2       Enable motor 1/2");
    Serial.println("  D1 / D2       Disable motor 1/2");
    Serial.println("  V1 <v>        Set motor1 voltage (e.g. V1 2.5)");
    Serial.println("  V2 <v>        Set motor2 voltage");
    Serial.println("  R             Read all sensors once");
    Serial.println("  M1 / M2      Toggle motor 1/2 monitoring (10Hz)");
    Serial.println("  I             Toggle IMU monitoring (10Hz)");
    Serial.println("  W             Toggle encoder watch (10Hz, auto on rotation)");
    Serial.println("  H1 / H2      Hold position (resist rotation)");
    Serial.println("  G <kp> <kd>   Set hold gains (default 3.0 0.3)");
    Serial.println("  C1 / C2       Auto-calibrate zero angle (sweep, ~25s)");
    Serial.println("  F1/F2 <offset> FOC zero angle offset (try 1.5708, -1.5708)");
    Serial.println("  K1 [Vmax total_ramp_ms print_hz]  Kt test motor1 (continuous ramp + regression)");
    Serial.println("  K2 [Vmax total_ramp_ms print_hz]  Kt test motor2");
    Serial.println("     defaults: 3.0V 60000ms total ramp, 20Hz CSV print");
    Serial.println("  L1 [volt]     Inductance test motor1 (default 2V)");
    Serial.println("  L2 [volt]     Inductance test motor2");
    Serial.println("  X1 [Vmax ramp_ms log_hz]  Rattle diagnostic motor1 (default 3.0V 30000ms 50Hz)");
    Serial.println("  X2 [Vmax ramp_ms log_hz]  Rattle diagnostic motor2");
    Serial.println("  S             Emergency stop all");
    Serial.println("  ?             This help");
}

// ── 명령 파서 ────────────────────────────────────────────────────────
void parse_command(String &cmd) {
    cmd.trim();
    String upper = cmd;
    upper.toUpperCase();

    if      (upper.startsWith("E1")) { motor1.enable();  Serial.println("OK: Motor1 enabled"); }
    else if (upper.startsWith("E2")) { motor2.enable();  Serial.println("OK: Motor2 enabled"); }
    else if (upper.startsWith("D1")) { motor1.target = 0; motor1.disable(); Serial.println("OK: Motor1 disabled"); }
    else if (upper.startsWith("D2")) { motor2.target = 0; motor2.disable(); Serial.println("OK: Motor2 disabled"); }
    else if (upper.startsWith("V1")) { float v = cmd.substring(2).toFloat(); motor1.target = v; Serial.printf("OK: M1 target=%.3fV\n", v); }
    else if (upper.startsWith("V2")) { float v = cmd.substring(2).toFloat(); motor2.target = v; Serial.printf("OK: M2 target=%.3fV\n", v); }
    else if (upper == "R")           { print_sensor_readings(); }
    else if (upper.startsWith("M1")) { monitoring_m1 = !monitoring_m1; Serial.printf("M1 monitor: %s\n", monitoring_m1 ? "ON" : "OFF"); }
    else if (upper.startsWith("M2")) { monitoring_m2 = !monitoring_m2; Serial.printf("M2 monitor: %s\n", monitoring_m2 ? "ON" : "OFF"); }
    else if (upper.startsWith("I"))  { imu_monitoring = !imu_monitoring; Serial.printf("IMU monitor: %s\n", imu_monitoring ? "ON" : "OFF"); }
    else if (upper.startsWith("W"))  { encoder_watching = !encoder_watching; Serial.printf("Encoder watch: %s\n", encoder_watching ? "ON" : "OFF"); }
    else if (upper.startsWith("H1")) {
        hold_m1 = !hold_m1;
        if (hold_m1) { encoder1.update(); hold_target1 = encoder1.getAngle(); motor1.enable(); }
        else { motor1.target = 0; motor1.disable(); }
        Serial.printf("Hold M1: %s (pos=%.3f Kp=%.1f Kd=%.2f)\n", hold_m1 ? "ON" : "OFF", hold_target1, hold_kp, hold_kd);
    }
    else if (upper.startsWith("H2")) {
        hold_m2 = !hold_m2;
        if (hold_m2) { encoder2.update(); hold_target2 = encoder2.getAngle(); motor2.enable(); }
        else { motor2.target = 0; motor2.disable(); }
        Serial.printf("Hold M2: %s (pos=%.3f Kp=%.1f Kd=%.2f)\n", hold_m2 ? "ON" : "OFF", hold_target2, hold_kp, hold_kd);
    }
    else if (upper.startsWith("G")) {
        String params = cmd.substring(1); params.trim();
        int sp = params.indexOf(' ');
        if (sp > 0) { hold_kp = params.substring(0, sp).toFloat(); hold_kd = params.substring(sp+1).toFloat(); }
        else if (params.length() > 0) { hold_kp = params.toFloat(); }
        Serial.printf("Hold gains: Kp=%.2f Kd=%.3f\n", hold_kp, hold_kd);
    }
    else if (upper.startsWith("F1")) {
        float offset = cmd.substring(2).toFloat();
        motor1.zero_electric_angle = foc_zero1 + offset;
        Serial.printf("M1 zero_angle: %.4f + %.4f = %.4f\n", foc_zero1, offset, motor1.zero_electric_angle);
    }
    else if (upper.startsWith("F2")) {
        float offset = cmd.substring(2).toFloat();
        motor2.zero_electric_angle = foc_zero2 + offset;
        Serial.printf("M2 zero_angle: %.4f + %.4f = %.4f\n", foc_zero2, offset, motor2.zero_electric_angle);
    }
    else if (upper.startsWith("A"))  {
        Serial.println("── Raw ADC (no SimpleFOC, 10 reads) ──");
        for (int i = 0; i < 10; i++) {
            Serial.printf("[%d] A0=%d  A1=%d  A2=%d  A3=%d\n",
                          i, analogRead(A0), analogRead(A1),
                          analogRead(A2), analogRead(A3));
            delay(200);
        }
    }
    else if (upper.startsWith("P"))  {
        Serial.println("── AS5048A SPI Diagnostic ──");
        int cs_pins[] = {ENC1_CS, ENC2_CS};
        for (int e = 0; e < 2; e++) {
            int cs = cs_pins[e];
            SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));

            // Read ANGLE register (0x3FFF)
            digitalWrite(cs, LOW);
            uint16_t angle_raw = SPI.transfer16(0xFFFF) & 0x3FFF;
            digitalWrite(cs, HIGH);
            delayMicroseconds(1);

            // Read DIAGNOSTICS + AGC register (0x3FFD)
            digitalWrite(cs, LOW);
            SPI.transfer16(0x7FFD); // command: read 0x3FFD
            digitalWrite(cs, HIGH);
            delayMicroseconds(1);
            digitalWrite(cs, LOW);
            uint16_t diag_raw = SPI.transfer16(0xC000); // NOP to clock out result
            digitalWrite(cs, HIGH);
            delayMicroseconds(1);

            // Read MAGNITUDE register (0x3FFE)
            digitalWrite(cs, LOW);
            SPI.transfer16(0x7FFE); // command: read 0x3FFE
            digitalWrite(cs, HIGH);
            delayMicroseconds(1);
            digitalWrite(cs, LOW);
            uint16_t mag_raw = SPI.transfer16(0xC000);
            digitalWrite(cs, HIGH);

            SPI.endTransaction();

            uint8_t agc = diag_raw & 0xFF;
            bool err_frame = (diag_raw >> 14) & 1;
            bool err_parity = (diag_raw >> 15) & 1;
            uint16_t magnitude = mag_raw & 0x3FFF;

            // Read angle 5 times to check if changing
            Serial.printf("ENC%d (CS=%d):\n", e+1, cs);
            Serial.printf("  Raw angle: %u (%.2f deg)\n", angle_raw, angle_raw * 360.0f / 16384.0f);
            Serial.printf("  AGC: %u (0=magnet too strong, 255=too weak)\n", agc);
            Serial.printf("  Magnitude: %u\n", magnitude);
            Serial.printf("  Errors: frame=%d parity=%d\n", err_frame, err_parity);

            Serial.print("  Angle x5: ");
            for (int i = 0; i < 5; i++) {
                SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
                digitalWrite(cs, LOW);
                uint16_t a = SPI.transfer16(0xFFFF) & 0x3FFF;
                digitalWrite(cs, HIGH);
                SPI.endTransaction();
                Serial.printf("%u ", a);
                delay(300);
            }
            Serial.println();
        }
    }
    else if (upper.startsWith("C1")) { run_zero_cal(motor1, 1); }
    else if (upper.startsWith("C2")) { run_zero_cal(motor2, 2); }
    else if (upper.startsWith("K1")) { run_kt_test(motor1, encoder1, current_sense1, cmd); }
    else if (upper.startsWith("K2")) { run_kt_test(motor2, encoder2, current_sense2, cmd); }
    else if (upper.startsWith("L1")) { run_inductance_test(motor1, driver1, current_sense1, cmd); }
    else if (upper.startsWith("L2")) { run_inductance_test(motor2, driver2, current_sense2, cmd); }
    else if (upper.startsWith("X1")) { run_rattle_diag(motor1, encoder1, current_sense1, cmd); }
    else if (upper.startsWith("X2")) { run_rattle_diag(motor2, encoder2, current_sense2, cmd); }
    else if (upper.startsWith("S"))  { emergency_stop(); }
    else if (upper.startsWith("?"))  { print_help(); }
    else { Serial.println("ERR: Unknown command. Type ? for help"); }
}

void process_serial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_buf.length() > 0) {
                parse_command(serial_buf);
                serial_buf = "";
            }
        } else {
            serial_buf += c;
        }
    }
}

// ── MOTOR_TEST setup / loop ─────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("================================");
    Serial.println(" GM4108 Dual Motor Test Firmware");
    Serial.println(" MacroBase Dual FOC 3.2 Plus");
    Serial.println("================================");

    // IMU 초기화
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(100);

    Serial.print("IMU init... ");
    bool imu_ok = false;
    for (int i = 0; i < 10; i++) {
        if (setup_icm20948()) { imu_ok = true; break; }
        delay(200);
    }
    Serial.println(imu_ok ? "OK" : "FAILED (continuing without IMU)");

    if (imu_ok) filter.begin(IMU_HZ);

    // 모터 초기화
    Serial.print("Motors init... ");
    setup_motors();
    Serial.println("OK");

    Serial.println("Type ? for commands");
    Serial.println("================================");
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    process_serial();

    if (hold_m1) {
        encoder1.update();
        float err = encoder1.getAngle() - hold_target1;
        float vel = encoder1.getVelocity();
        float v = hold_kp * err + hold_kd * vel;
        v = constrain(v, -MOTOR_VLIMIT, MOTOR_VLIMIT);
        motor1.target = v;
    }
    if (hold_m2) {
        encoder2.update();
        float err = hold_target2 - encoder2.getAngle();
        float vel = encoder2.getVelocity();
        float v = hold_kp * err - hold_kd * vel;
        v = constrain(v, -MOTOR_VLIMIT, MOTOR_VLIMIT);
        motor2.target = v;
    }

    if (motor1.enabled) { motor1.loopFOC(); motor1.move(); }
    if (motor2.enabled) { motor2.loopFOC(); motor2.move(); }

    unsigned long now = millis();

    // 모터/엔코더 모니터링 (10Hz)
    if ((monitoring_m1 || monitoring_m2 || encoder_watching) && now - last_monitor_ms >= 100) {
        last_monitor_ms = now;
        if (monitoring_m1) {
            Serial.printf("M1: ang=%.3f vel=%.3f Iq=%.4f Id=%.4f\n",
                encoder1.getAngle(), motor1.shaft_velocity,
                motor1.current.q, motor1.current.d);
        }
        if (monitoring_m2) {
            Serial.printf("M2: ang=%.3f vel=%.3f Iq=%.4f Id=%.4f\n",
                encoder2.getAngle(), motor2.shaft_velocity,
                motor2.current.q, motor2.current.d);
        }
        if (encoder_watching) {
            encoder1.update();
            encoder2.update();
            float v1 = encoder1.getVelocity();
            float v2 = encoder2.getVelocity();
            if (fabsf(v1) > 0.05f || fabsf(v2) > 0.05f) {
                Serial.printf("ENC: e1=%.3f(%.2f) e2=%.3f(%.2f) rad\n",
                    encoder1.getAngle(), v1,
                    encoder2.getAngle(), v2);
            }
        }
    }

    // IMU 모니터링 (10Hz)
    if (imu_monitoring && now - last_imu_ms >= 100) {
        last_imu_ms = now;
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float mx_cal = (myICM.magX() - mag_offset[0]) * mag_scale[0];
            float my_cal = (myICM.magY() - mag_offset[1]) * mag_scale[1];
            float mz_cal = (myICM.magZ() - mag_offset[2]) * mag_scale[2];
            filter.update(myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
                          myICM.accX()/1000.0f, myICM.accY()/1000.0f, myICM.accZ()/1000.0f,
                          mx_cal, my_cal, mz_cal);
            Serial.printf("IMU: R=%.1f P=%.1f Y=%.1f  ax=%.1f ay=%.1f az=%.1f mg\n",
                filter.getRoll(), filter.getPitch(), filter.getYaw(),
                myICM.accX(), myICM.accY(), myICM.accZ());
        }
    }
}

#endif // MOTOR_TEST_MODE

// ═══════════════════════════════════════════════════════════════════════
// ╔═══════════════════════════════════════════════════════════════════╗
// ║                         ROS_MODE                                ║
// ╚═══════════════════════════════════════════════════════════════════╝
// ═══════════════════════════════════════════════════════════════════════
#ifdef ROS_MODE

// ── micro-ROS 객체 ──────────────────────────────────────────────────
rcl_node_t          node;
rcl_allocator_t     allocator;
rclc_support_t      support;
rclc_executor_t     executor;

// IMU publisher
rcl_publisher_t     imu_pub;
sensor_msgs__msg__Imu imu_msg;
static char imu_frame_id[16] = "imu_link";

// Motor state publisher
rcl_publisher_t     motor_pub;
sensor_msgs__msg__JointState motor_state_msg;
static char motor_frame_id[16] = "base_link";
static char* joint_names[2];
static char jn0[16] = "wrist_pitch";
static char jn1[16] = "wrist_roll";
static double js_pos[2], js_vel[2], js_eff[2];
static rosidl_runtime_c__String js_name_buf[2];

// Motor command subscriber
rcl_subscription_t  motor_cmd_sub;
std_msgs__msg__Float32MultiArray motor_cmd_msg;
static float cmd_data_buf[2] = {0, 0};

unsigned long last_pub_ms = 0;
int pub_fail_count = 0;
#define PUB_FAIL_LIMIT 100

// ── 에러 처리 ────────────────────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ── 모터 명령 콜백 ──────────────────────────────────────────────────
void motor_cmd_callback(const void *msg_in) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size >= 1) {
        motor1.target = msg->data.data[0];
        if (!motor1.enabled) motor1.enable();
    }
    if (msg->data.size >= 2) {
        motor2.target = msg->data.data[1];
        if (!motor2.enabled) motor2.enable();
    }
}

// ── IMU 발행 ─────────────────────────────────────────────────────────
void publish_imu(float ax_ms2, float ay_ms2, float az_ms2,
                 float gx_rads, float gy_rads, float gz_rads) {
    int64_t ms = rmw_uros_epoch_millis();
    imu_msg.header.stamp.sec     = (int32_t)(ms / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

    float qw, qx, qy, qz;
    eulerToQuaternion(filter.getRollRadians(), filter.getPitchRadians(),
                      filter.getYawRadians(), qw, qx, qy, qz);
    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;

    imu_msg.angular_velocity.x = gx_rads;
    imu_msg.angular_velocity.y = gy_rads;
    imu_msg.angular_velocity.z = gz_rads;

    imu_msg.linear_acceleration.x = ax_ms2;
    imu_msg.linear_acceleration.y = ay_ms2;
    imu_msg.linear_acceleration.z = az_ms2;

    rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        pub_fail_count++;
        if (pub_fail_count >= PUB_FAIL_LIMIT) SCB_AIRCR = 0x05FA0004;
    } else {
        pub_fail_count = 0;
    }
}

// ── 모터 상태 발행 ───────────────────────────────────────────────────
void publish_motor_state() {
    int64_t ms = rmw_uros_epoch_millis();
    motor_state_msg.header.stamp.sec     = (int32_t)(ms / 1000);
    motor_state_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

    js_pos[0] = encoder1.getAngle();
    js_pos[1] = encoder2.getAngle();
    js_vel[0] = motor1.shaft_velocity;
    js_vel[1] = motor2.shaft_velocity;
    js_eff[0] = motor1.current.q * GM4108_KT;  // torque [N·m]
    js_eff[1] = motor2.current.q * GM4108_KT;

    rcl_publish(&motor_pub, &motor_state_msg, NULL);
}

// ── ROS_MODE setup ──────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // I2C + IMU
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(100);

    bool imu_ok = false;
    for (int i = 0; i < 20; i++) {
        if (setup_icm20948()) { imu_ok = true; break; }
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
    }
    if (!imu_ok) {
        while (true) {
            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_BUILTIN, HIGH); delay(80);
                digitalWrite(LED_BUILTIN, LOW);  delay(80);
            }
            delay(1000);
        }
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    filter.begin(IMU_HZ);

    // Motors
    setup_motors();

    // micro-ROS
    set_microros_transports();
    delay(2000);

    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 5));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "imu_motor_teensy", "", &support));

    // IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"));

    imu_msg.header.frame_id.data     = imu_frame_id;
    imu_msg.header.frame_id.size     = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

    memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;

    memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
    imu_msg.angular_velocity_covariance[0] = 6.8e-6;
    imu_msg.angular_velocity_covariance[4] = 6.8e-6;
    imu_msg.angular_velocity_covariance[8] = 6.8e-6;

    memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
    imu_msg.linear_acceleration_covariance[0] = 0.005;
    imu_msg.linear_acceleration_covariance[4] = 0.005;
    imu_msg.linear_acceleration_covariance[8] = 0.005;

    // Motor state publisher (JointState)
    RCCHECK(rclc_publisher_init_default(
        &motor_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/motor_states"));

    motor_state_msg.header.frame_id.data     = motor_frame_id;
    motor_state_msg.header.frame_id.size     = strlen(motor_frame_id);
    motor_state_msg.header.frame_id.capacity = sizeof(motor_frame_id);

    js_name_buf[0].data = jn0; js_name_buf[0].size = strlen(jn0); js_name_buf[0].capacity = sizeof(jn0);
    js_name_buf[1].data = jn1; js_name_buf[1].size = strlen(jn1); js_name_buf[1].capacity = sizeof(jn1);
    motor_state_msg.name.data = js_name_buf;
    motor_state_msg.name.size = 2;
    motor_state_msg.name.capacity = 2;

    motor_state_msg.position.data = js_pos;
    motor_state_msg.position.size = 2;
    motor_state_msg.position.capacity = 2;
    motor_state_msg.velocity.data = js_vel;
    motor_state_msg.velocity.size = 2;
    motor_state_msg.velocity.capacity = 2;
    motor_state_msg.effort.data = js_eff;
    motor_state_msg.effort.size = 2;
    motor_state_msg.effort.capacity = 2;

    // Motor command subscriber (Float32MultiArray)
    RCCHECK(rclc_subscription_init_default(
        &motor_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/motor_commands"));

    motor_cmd_msg.data.data = cmd_data_buf;
    motor_cmd_msg.data.size = 0;
    motor_cmd_msg.data.capacity = 2;

    // Executor: 1 subscription
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_sub,
            &motor_cmd_msg, &motor_cmd_callback, ON_NEW_DATA));

    rmw_uros_sync_session(1000);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ── ROS_MODE loop ───────────────────────────────────────────────────
void loop() {
    motor1.loopFOC();
    motor1.move();
    motor2.loopFOC();
    motor2.move();

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    unsigned long now = millis();
    if (now - last_pub_ms >= IMU_PERIOD_MS) {
        last_pub_ms = now;

        if (myICM.dataReady()) {
            myICM.getAGMT();

            float mx_cal = (myICM.magX() - mag_offset[0]) * mag_scale[0];
            float my_cal = (myICM.magY() - mag_offset[1]) * mag_scale[1];
            float mz_cal = (myICM.magZ() - mag_offset[2]) * mag_scale[2];

            filter.update(myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
                          myICM.accX()/1000.0f, myICM.accY()/1000.0f, myICM.accZ()/1000.0f,
                          mx_cal, my_cal, mz_cal);

            float ax_ms2  = myICM.accX() * 9.80665f / 1000.0f;
            float ay_ms2  = myICM.accY() * 9.80665f / 1000.0f;
            float az_ms2  = myICM.accZ() * 9.80665f / 1000.0f;
            float gx_rads = myICM.gyrX() * (PI / 180.0f);
            float gy_rads = myICM.gyrY() * (PI / 180.0f);
            float gz_rads = myICM.gyrZ() * (PI / 180.0f);

            publish_imu(ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads);
        }

        publish_motor_state();
    }
}

#endif // ROS_MODE
