/**
 * hw_selftest_esp32.ino  -  LOLIN D32 (ESP32)
 * Integrated GimbalArm self-test / bring-up firmware. Merges the four verified
 * standalone sketches onto ONE board + ONE menu:
 *   - enc_imu_spi_test      : 2x AS5048A + ICM20948 on shared SPI bus
 *   - motor_test_esp32      : GM4108 x2 FOC (spin / current raw / auto / recal)
 *   - gimbal_level_hold_esp32: 1-axis self-leveling PD hold on motor2 (IMU z)
 *   - can_test_esp32        : AK40-10 QDD over CAN/TWAI (status / impedance hold)
 *
 * All code pulled 1:1 from those verified sketches (no reimplementation).
 *
 * ---- VERIFIED PIN MAP (LOLIN D32 real silk, 2026-06-12) --------------------
 *   GM4108 M0 PWM A/B/C = 25/26/27   EN0 = 14
 *   GM4108 M1 PWM A/B/C = 16/17/4    EN1 = 12   (strapping: onboard short only)
 *   Current sense (ADC1): M0 = 36/39   M1 = 34/32
 *   Shared SPI: SCK=18 MISO=19 MOSI=23  ENC1_CS=21 ENC2_CS=22 IMU_CS=13
 *   IMU ICM20948 on SPI: SCL=SCK SDA=MOSI AD0=MISO NCS=IMU_CS
 *   CAN TWAI: TX=5 RX=33  -> SN65HVD230 -> AK40 ids 1,2  (120ohm both ends)
 *   Star ground: motor power GND must NOT share the 70cm LAN/SPI GND.
 * ----------------------------------------------------------------------------
 *
 * WARNING: ESP32 GPIO are 3.3V max, NOT 5V tolerant. Keep star ground; motor
 *   return current through a shared GND killed a Teensy (ground-offset ESD).
 *
 * Bring-up order: PSU 20V CURRENT LIMIT ~1.5A first (align needs ~1.1A). Boot does FOC align
 *   (motors twitch) + 2s gyro bias calib (KEEP STILL). Current-sense offset is
 *   calibrated at boot -- if MKS 12V was OFF then, run 'c' recal after powering.
 *
 * Serial @115200. Keys:
 *   2 IMU live          3 Encoder check     4 Current raw
 *   9 Current AUTO ('9'->'1'/'2')           c recal current offsets
 *   5 spin M0           6 spin M1           (GM4108 closed-loop vel)
 *   l level-hold        (capture phi_z + PD hold on M1/IMU z)
 *   7 AK40 status       8 AK40 impedance+ESO hold ('8'->'1'/'2')
 *     j J-test id1   o origin('o'->id)   ESO(in hold): B comp  G<hz>bw  A<val>auth  E<val>setJ  I<val>Ki
 *   x STOP -> menu      0/? this menu
 *
 * Libraries: SimpleFOC, SparkFun ICM_20948, ESP32 core (driver/twai.h)
 */

#include <SimpleFOC.h>
#include <SPI.h>
#include <ICM_20948.h>
#include "driver/twai.h"

// ============================================================================
// Pins (verified)
// ============================================================================
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

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define ENC1_CS  21
#define ENC2_CS  15   // moved off GPIO22 (D32 internal GPIO22<->23/MOSI short). 22 left undriven.
#define IMU_CS   13

#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_33

// ============================================================================
// Params
// ============================================================================
#define GM4108_PP        11
#define GM4108_PHASE_R   5.5f
// 20V bus (2026-06-17): R=5.5ohm is a high-resistance / low-current winding, so stall
// torque is current-limited and current needs voltage headroom (Iq=Uq/R at stall,
// Uq_max ~= 0.5..0.577*Vbus).  12V topped out ~1.2A; 20V allows ~1.8-2.1A (~2x torque).
// voltage_power_supply MUST match the real bus or the duty->volt scaling over-drives.
// NOTE: 1.8A through 5.5ohm is a lot of I^2R for a small gimbal motor -> treat as PEAK
// (dynamic leveling), not continuous hold.  Watch motor temp; sustained high current
// cooks it (same load-heat lesson as the AK40 shoulder).
#define SUPPLY_VOLTAGE   20.0f
#define DRIVER_VLIMIT    12.0f
#define MOTOR_VLIMIT     10.0f   // 10/5.5 ~= 1.8A peak; lower if it overheats
#define MOTOR_ILIMIT     2.0f
#define MOTOR_VEL_LIMIT  20.0f
#define CS_SHUNT         0.01f
#define CS_GAIN          50.0f
#define SPIN_VEL_RADS    5.0f
#define ENC_SPI_HZ       1000000UL   // back to 1M (failure was D32 GPIO22<->23 short, not SI;
#define IMU_SPI_HZ       1000000UL   // fixed by remapping ENC2_CS 22->15)

// leveling (z-axis, motor2) -- gains from project_gimbal_axis_mapping (Kp=4 ok)
#define USE_ACCEL
float Kp = 4.0f;
float Kd = 0.25f;
#define COMP_ALPHA     0.98f
#define SIGN_X         (-1.0f)        // flip if motor2 leveling diverges
#define SOFT_LIMIT_RAD 1.20f          // |enc2 - enc2_at_capture| e-stop (~40deg)
#define IMU_HZ         100
#define IMU_PERIOD_US  (1000000UL / IMU_HZ)

// AK40
#define M1_CAN_ID 1
#define M1_KT     0.0705f
#define M1_J_NOM  1.7e-3f    // measured joint inertia, no arm; re-measure with arm
#define M2_CAN_ID 2
#define M2_KT     0.0908f
#define M2_J_NOM  2.11e-3f   // measured joint inertia, no arm
#define GEAR_RATIO 10.0f
#define GEAR_EFF   0.90f
#define POLE_PAIRS 14
#define POS_AT_JOINT 1
#define POS_SCALE  10000.0f     // SET_POS payload = deg * this (measured: 1e6 -> 100x overshoot)
#define AK40_NUM  2
#define MIT_KP_DEF 20.0f     // MIT native stiffness default [Nm/rad] (start soft, ramp up)
#define MIT_KD_DEF 0.5f      // MIT native damping default [Nm/(rad/s)]
#define K_IMP     2.0f
#define D_IMP     0.05f
#define KI_IMP    0.0f       // integral gain [Nm/(rad*s)]; runtime-tunable via 'I<val>' in hold
#define INTEG_TAU_MAX 0.5f   // integral torque clamp [Nm] (anti-windup)
#define I_MAX     2.0f
#define POS_ERR_LIMIT_DEG 60.0f
#define AK40_CTRL_HZ 200
#define AK40_CTRL_US (1000000UL / AK40_CTRL_HZ)
#define STATUS_PKT 0x29
#define DEG2RAD  0.0174532925f
#define RPM2RADS 0.1047197551f

typedef enum {
  CAN_PACKET_SET_CURRENT     = 1,
  CAN_PACKET_SET_POS         = 4,
  CAN_PACKET_SET_ORIGIN_HERE = 5,
} CAN_PACKET_ID;

// ============================================================================
// Objects
// ============================================================================
BLDCMotor motor1 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCMotor motor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M0_A, M0_B, M0_C, M0_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M1_A, M1_B, M1_C, M1_EN);

MagneticSensorSPIConfig_s AS5048_CFG = {
  .spi_mode = SPI_MODE1, .clock_speed = ENC_SPI_HZ, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_CFG, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_CFG, ENC2_CS);

InlineCurrentSense cs1 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CUR0_A, CUR0_B);
InlineCurrentSense cs2 = InlineCurrentSense(CS_SHUNT, CS_GAIN, CUR1_A, CUR1_B);

ICM_20948_SPI myICM;

bool m1_ok = false, m2_ok = false, imu_ok = false, can_ok = false;
float foc_zero1 = 0, foc_zero2 = 0;

// leveling state (phi_z = tilt around IMU z)
float est_phi = 0, gyro_bias[3] = {0, 0, 0};
float target_phi = 0, enc1_at_C = 0;
bool  holding = false;
unsigned long last_imu_us = 0;

struct AK40Motor {
  uint8_t can_id; float Kt, dir;
  float theta, omega; bool haveStatus;
  float theta_d, K, D, Ki; bool enabled;
  float omega_f, dbg_I, integ, dbg_taui;
  // ESO (Level3 DOB): states [theta, omega, f=d/J] + disturbance torque estimate
  float J_nom, eso_x1, eso_x2, eso_x3, d_hat, u_app;
  float I_meas, adm_x0;          // admittance: status motor current [A], anchor pos [rad]
  float adm_If, adm_dx;          // admittance: LPF'd current [A], yield offset state [rad]
  float adm_I0; bool adm_cal; uint32_t adm_t0;  // gravity-hold current baseline + 1-shot calib
  // MIT native impedance (motor onboard loop): kp[Nm/rad] kd[Nm/(rad/s)] p_des[rad] tff[Nm]
  float mit_kp, mit_kd, mit_pdes, mit_tff, mit_tau; bool mit_have;
  float mit_pgoal, mit_pvel;     // T/t goal [rad] + profile vel [rad/s]; mit_pdes ramps here (vel/accel cap)
};
AK40Motor ak[AK40_NUM];
int ak_sel = 0;
bool origin_pending = false;   // 'o' armed -> next '1'/'2' sets that id's origin
bool jtest_pending  = false;   // 'j' armed -> next '1'/'2' runs J-test on that id
bool mit_pending    = false;   // 'm' armed -> next '1'/'2' runs MIT-mode probe on that id
bool mit_dual       = false;   // 'n'+'3' -> BOTH AK40 held in MIT; T<deg> jogs both relatively
// MIT jog profile: T/t set a GOAL; mit_pdes ramps to it under vel+accel caps
// (trapezoid). kills the position STEP that surged ~20A and faulted the motor.
// CONSERVATIVE defaults -- raise live via 'S'(speed) / 'R'(accel) once trusted.
float mit_vmax   = 20.0f * DEG2RAD;   // 'S<deg/s>'  max jog speed  [rad/s]
float mit_amax   = 40.0f * DEG2RAD;   // 'R<deg/s2>' max jog accel  [rad/s^2]
// jog damping boost: while ramping (profile moving), raise kd to mit_kd_jog so the
// transit is damped; restore the (low) hold kd when stopped. keeps D0 hold smooth.
float mit_kd_jog = 0.1f;              // 'Z<val>' boost damping during jog [Nm/(rad/s)]

// ---- AK40 ESO (Extended State Observer = Level3 DOB, mirror gimbal ESO) ------
//   plant (joint frame): w_dot = b0*u + f,  b0 = kt_tot/J,  u = applied joint current.
//   x1'=x2+l1*e, x2'=b0*u+x3+l2*e, x3'=l3*e   (e=theta-x1). Triple pole -wo (Gao):
//   l1=3wo, l2=3wo^2, l3=wo^3.  d_hat = J*x3 [N·m joint].  ONE knob wo.
//   Torque domain (CAN direct current): no R/Kt voltage map needed vs gimbal.
#define AK_J_NOM_DEF  0.0015f      // joint inertia [kg·m^2] default; set via 'E<val>' after 'j' meas
#define AK_DOB_TCLAMP 1.0f         // |comp torque| safety clamp [N·m]
bool  ak_dob_on    = false;        // 'B' gate compensation (ESO always runs)
float ak_dob_scale = 0.7f;         // 'A' comp authority (signed; + = correct sign, VERIFY on HW)
float ak_dither_amp = 0.0f;        // 'Y' dither current amplitude [A] (0=off) -- breaks stick-slip
float ak_dither_hz  = 50.0f;       // 'F' dither frequency [Hz]
float ak_dither_ph  = 0.0f;        // dither phase accumulator
float ak_eso_wo    = 2.0f * PI * 5.0f;  // 'G<hz>' observer bw [rad/s]; keep < ~9Hz mech resonance
float ak_eso_l1 = 0, ak_eso_l2 = 0, ak_eso_l3 = 0;

// ---- AK40 admittance (variable stiffness via VESC SET_POS inner loop) --------
//   inner = VESC onboard position loop (SET_POS) -> smooth, kills gearbox stick-slip.
//   outer (ESP32, this firmware): read motor current from status -> tau_ext=kt_tot*I,
//   yield position  x_cmd = x0 + sign*tau_ext/Kv.  Kv = virtual stiffness (runtime-
//   tunable = VARIABLE STIFFNESS). No dither, no end-link buzz.  ONE knob: Kv.
float adm_Kv   = 2.0f;          // virtual stiffness [N·m/rad]; 'V<val>' (smaller = softer)
float adm_tau  = 0.3f;          // yield/return time const [s]; 'W<val>' (independent of Kv)
float adm_sign = 1.0f;          // compliance direction; 'N' toggles (VERIFY on HW)
#define ADM_I_DB       0.08f    // current deadband [A] -- reject status noise so it won't drift
#define ADM_I_FC       15.0f    // current LPF cutoff [Hz] -- kill 200Hz status chatter
#define ADM_DX_MAX_DEG 30.0f    // clamp on yield offset [deg] (safety)
#define ADM_CAL_MS     350      // settle time after anchor before grabbing gravity baseline [ms]
#define ADM_OM_GATE    0.6f     // [rad/s] velocity gate: freeze dx growth above this (anti-runaway)

enum Mode { MENU, IMU_LIVE, ENC_CHECK, CURR_CHECK, CURR_AUTO,
            SPIN_M0, SPIN_M1, LEVEL_HOLD, AK40_STATUS, AK40_HOLD, AK40_POS, AK40_ADM, AK40_MIT };
Mode mode = MENU;
String serial_buf = "";
unsigned long last_print_ms = 0;

void set_ak_eso_bw(float wo) { ak_eso_wo = wo; ak_eso_l1 = 3*wo; ak_eso_l2 = 3*wo*wo; ak_eso_l3 = wo*wo*wo; }
static inline void ak_eso_seed(AK40Motor &m) {
  m.eso_x1 = m.theta; m.eso_x2 = m.omega; m.eso_x3 = 0; m.d_hat = 0; m.u_app = 0;
}

// ============================================================================
// TWAI (CAN)
// ============================================================================
static bool twai_setup() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32; g.tx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}
static void canSend(uint8_t id, CAN_PACKET_ID pkt, const uint8_t *d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = ((uint32_t)pkt << 8) | id;
  m.extd = 1;
  m.data_length_code = len;
  for (uint8_t k = 0; k < len; k++) m.data[k] = d[k];
  twai_transmit(&m, pdMS_TO_TICKS(1));
}
static void buf_append_int32(uint8_t *b, int32_t v, int *i) {
  b[(*i)++] = v >> 24; b[(*i)++] = v >> 16; b[(*i)++] = v >> 8; b[(*i)++] = v;
}
static int16_t buf_get_int16(const uint8_t *b, int o) {
  return (int16_t)((b[o] << 8) | b[o + 1]);
}
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
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
static void motorSetPos(uint8_t id, float deg) {
  uint8_t b[4]; int i = 0;
  buf_append_int32(b, (int32_t)(deg * POS_SCALE), &i);
  canSend(id, CAN_PACKET_SET_POS, b, i);
}
// ---- MIT mode probe (standard 11-bit frame) --------------------------------
// AK servo firmware decodes MIT frames only if the run-mode flashed via R-Link
// enables it. Probe: (0) servo poll baseline, (1) MIT enter, (2) zero-torque MIT
// frame, dumping every frame and classifying servo status (ext, pkt 0x29) vs a
// MIT reply (standard frame). MIT reply => MIT supported. Servo-only reply =>
// servo firmware (MIT off). Silence even to servo poll => bus/power problem.
#define MIT_P_MIN -12.5f
#define MIT_P_MAX  12.5f
#define MIT_V_MIN -65.0f
#define MIT_V_MAX  65.0f
#define MIT_KP_MAX 500.0f
#define MIT_KD_MAX 5.0f
#define MIT_T_MIN -18.0f
#define MIT_T_MAX  18.0f

static uint16_t float_to_uint(float x, float lo, float hi, int bits) {
  if (x < lo) x = lo; else if (x > hi) x = hi;
  return (uint16_t)((x - lo) * (float)((1u << bits) - 1u) / (hi - lo));
}
static void canSendStd(uint8_t id, const uint8_t *d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = id;            // standard 11-bit, ID = motor id
  m.extd = 0;
  m.data_length_code = len;
  for (uint8_t k = 0; k < len; k++) m.data[k] = d[k];
  twai_transmit(&m, pdMS_TO_TICKS(2));
}
static void mit_special(uint8_t id, uint8_t code) {   // FC enter, FD exit, FE zero
  uint8_t b[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, code};
  canSendStd(id, b, 8);
}
static void mit_pack(uint8_t id, float p, float v, float kp, float kd, float t) {
  uint16_t pi = float_to_uint(p,  MIT_P_MIN, MIT_P_MAX, 16);
  uint16_t vi = float_to_uint(v,  MIT_V_MIN, MIT_V_MAX, 12);
  uint16_t ki = float_to_uint(kp, 0.0f,      MIT_KP_MAX, 12);
  uint16_t di = float_to_uint(kd, 0.0f,      MIT_KD_MAX, 12);
  uint16_t ti = float_to_uint(t,  MIT_T_MIN, MIT_T_MAX, 12);
  uint8_t b[8];
  b[0] = pi >> 8;
  b[1] = pi & 0xFF;
  b[2] = vi >> 4;
  b[3] = ((vi & 0xF) << 4) | (ki >> 8);
  b[4] = ki & 0xFF;
  b[5] = di >> 4;
  b[6] = ((di & 0xF) << 4) | (ti >> 8);
  b[7] = ti & 0xFF;
  canSendStd(id, b, 8);
}
static float uint_to_float(uint16_t x, float lo, float hi, int bits) {
  return lo + (float)x * (hi - lo) / (float)((1u << bits) - 1u);
}
// MIT reply (standard frame): b0=id, b1..b2=pos16, b3=vel hi8,
//   b4=(vel lo4)<<4|(cur hi4), b5=cur lo8.  pos[rad] vel[rad/s] cur=torque[Nm], at OUTPUT.
static void parseMitReply(const twai_message_t &msg) {
  if (msg.extd || msg.data_length_code < 6) return;     // VESC status is ext -> skip here
  uint8_t id = msg.data[0];
  for (int k = 0; k < AK40_NUM; k++) {
    if (ak[k].can_id != id) continue;
    uint16_t pi = ((uint16_t)msg.data[1] << 8) | msg.data[2];
    uint16_t vi = ((uint16_t)msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t ci = (((uint16_t)(msg.data[4] & 0xF)) << 8) | msg.data[5];
    ak[k].theta   = uint_to_float(pi, MIT_P_MIN, MIT_P_MAX, 16);
    ak[k].omega   = uint_to_float(vi, MIT_V_MIN, MIT_V_MAX, 12);
    ak[k].mit_tau = uint_to_float(ci, MIT_T_MIN, MIT_T_MAX, 12);
    ak[k].haveStatus = true; ak[k].mit_have = true;
    return;
  }
}
static int  g_dump_left = 0;          // live-dump budget for this phase (anti-flood)
static bool g_have_mit_sample = false; // captured first MIT-candidate frame?
static char g_mit_sample[80];
static void fmt_frame(char *out, int n, const twai_message_t &m) {
  int o = snprintf(out, n, "%s id=0x%03X dlc=%u:", m.extd ? "EXT" : "STD",
                   (unsigned)m.identifier, m.data_length_code);
  for (int i = 0; i < m.data_length_code && o < n - 4; i++)
    o += snprintf(out + o, n - o, " %02X", m.data[i]);
}
static void dump_frame(const twai_message_t &m) {
  if (g_dump_left <= 0) return;        // budget spent -> count only, no print (anti-flood)
  g_dump_left--;
  char line[80]; fmt_frame(line, sizeof(line), m);
  Serial.printf("  RX %s\n", line);
  if (g_dump_left == 0) Serial.println("  ...(more frames suppressed; see counts below)");
}
static int classify_rx(const twai_message_t &m, int *servo) {
  dump_frame(m);
  if (m.extd && ((m.identifier >> 8) & 0xFF) == STATUS_PKT) { (*servo)++; return 0; }
  if (!g_have_mit_sample) {            // keep first MIT-candidate frame for the verdict
    fmt_frame(g_mit_sample, sizeof(g_mit_sample), m); g_have_mit_sample = true;
  }
  return 1;   // standard frame, or non-status ext = MIT-candidate reply
}
// Safe: kp=kd=t=0 -> commanded torque 0 -> motor stays limp through the probe.
static void mit_probe(uint8_t id) {
  if (!can_ok) { Serial.println("CAN not up, probe refused"); return; }
  Serial.printf("\n--- MIT PROBE AK%u (zero torque, will not move) ---\n", id);
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {}            // flush stale rx
  int mit = 0, servo = 0;
  g_have_mit_sample = false;

  Serial.println("[0] servo poll (SET_CURRENT 0A, ext) -- bus/servo baseline:");
  g_dump_left = 6;                                     // dump only first 6 frames/phase (anti-flood)
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
    motorSetCurrent(id, 0.0f);
    unsigned long s = millis();
    while (millis() - s < 30)
      if (twai_receive(&rx, 0) == ESP_OK) classify_rx(rx, &servo);
  }

  while (twai_receive(&rx, 0) == ESP_OK) {}            // flush servo replies
  Serial.println("[1] MIT enter motor mode: STD id, data=FF FF FF FF FF FF FF FC");
  g_dump_left = 6;
  mit_special(id, 0xFC);
  t0 = millis();
  while (millis() - t0 < 400)
    if (twai_receive(&rx, 0) == ESP_OK) mit += classify_rx(rx, &servo);

  Serial.println("[2] MIT zero-torque frame (p=v=kp=kd=t=0), repeated:");
  g_dump_left = 6;
  t0 = millis();
  while (millis() - t0 < 600) {
    mit_pack(id, 0, 0, 0, 0, 0);
    unsigned long s = millis();
    while (millis() - s < 40)
      if (twai_receive(&rx, 0) == ESP_OK) mit += classify_rx(rx, &servo);
  }

  Serial.println("[3] MIT exit: data=FF FF FF FF FF FF FF FD");
  g_dump_left = 6;
  mit_special(id, 0xFD);
  delay(20);
  while (twai_receive(&rx, 0) == ESP_OK) classify_rx(rx, &servo);

  Serial.printf("--- PROBE done: MIT-reply=%d  servo-status=%d ---\n", mit, servo);
  if (g_have_mit_sample) Serial.printf("  MIT-candidate sample: %s\n", g_mit_sample);
  if (mit > 0)
    Serial.println("  => MIT DECODED: firmware accepts MIT frames. Implement MIT mode.");
  else if (servo > 0)
    Serial.println("  => servo replies, NO MIT reply: servo-only firmware (MIT off). Go admittance.");
  else
    Serial.println("  => SILENCE even to servo poll: bus/power/wiring, not a MIT verdict. Check 12V/term/CANH-L/baud.");
}

static void parseStatus(const twai_message_t &msg) {
  if (!msg.extd) return;
  if (((msg.identifier >> 8) & 0xFF) != STATUS_PKT) return;
  uint8_t id = msg.identifier & 0xFF;
  if (msg.data_length_code != 8) return;
  for (int k = 0; k < AK40_NUM; k++) {
    if (ak[k].can_id != id) continue;
    float pos_deg = buf_get_int16(msg.data, 0) * 0.1f;
    float erpm    = buf_get_int16(msg.data, 2) * 10.0f;
    if (!POS_AT_JOINT) pos_deg /= GEAR_RATIO;
    ak[k].theta = pos_deg * DEG2RAD;
    ak[k].omega = (erpm / (float)POLE_PAIRS / GEAR_RATIO) * RPM2RADS;
    ak[k].I_meas = buf_get_int16(msg.data, 4) * 0.01f;   // AK servo status: current = int16/100 [A]
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
  float kt_tot = m.Kt * GEAR_RATIO * GEAR_EFF;

  // ESO update: from joint theta + last applied joint current (b0*u = tau_applied/J)
  float b0  = kt_tot / m.J_nom;
  float err = th - m.eso_x1;
  m.eso_x1 += dt * (m.eso_x2 + ak_eso_l1 * err);
  m.eso_x2 += dt * (b0 * m.u_app + m.eso_x3 + ak_eso_l2 * err);
  m.eso_x3 += dt * (ak_eso_l3 * err);
  m.d_hat   = m.J_nom * m.eso_x3;                 // joint disturbance torque [N·m]

  // impedance PD + integral + ESO disturbance compensation
  float perr = m.theta_d - th;
  float tau_i = 0.0f;
  if (m.Ki > 1e-9f) {                            // integral w/ back-calc anti-windup
    m.integ += perr * dt;
    tau_i = m.Ki * m.integ;
    float tlim = clampf(tau_i, -INTEG_TAU_MAX, INTEG_TAU_MAX);
    if (tlim != tau_i) { m.integ = tlim / m.Ki; tau_i = tlim; }
  } else m.integ = 0.0f;
  float tau = m.K * perr + m.D * (0.0f - m.omega_f) + tau_i;
  if (ak_dob_on) tau += clampf(-ak_dob_scale * m.d_hat, -AK_DOB_TCLAMP, AK_DOB_TCLAMP);

  float I = clampf(m.dir * tau / kt_tot, -I_MAX, I_MAX);
  if (ak_dither_amp > 1e-6f) I = clampf(I + ak_dither_amp * sinf(ak_dither_ph), -I_MAX, I_MAX);  // stick-slip dither
  motorSetCurrent(m.can_id, I);
  m.dbg_I = I; m.dbg_taui = tau_i;
  m.u_app = m.dir * I;                            // joint-frame current actually applied (post-clamp)
}

// ---- AK40 admittance step: SET_POS inner loop + current-based yield outer ------
//   I_ext = LPF(I_meas) - I0(gravity hold), deadbanded.  tau_ext = kt_tot*I_ext.
//   1st-order lag toward compliance target:  dx += (gate*dx_tgt - dx)/tau * dt,
//     dx_tgt = sign*tau_ext/Kv.  Return time const = tau, INDEPENDENT of Kv (no
//   more B/Kv blowup at soft Kv).  I0 capture nulls gravity offset.
//   gate = 1/(1+(omega/OM_GATE)^2): the read current includes VESC effort to track
//   our OWN moving cmd (acceleration current) -> positive feedback -> runaway to
//   clamp.  gate freezes dx growth while the joint moves fast (= the runaway regime)
//   and only yields when quasi-static (true external hold).  Kills the runaway.
static void ak_admStep(AK40Motor &m, float dt) {
  if (!m.enabled || !m.haveStatus) return;
  float kt_tot = m.Kt * GEAR_RATIO * GEAR_EFF;
  float a = 2.0f * PI * ADM_I_FC * dt; a = a / (1.0f + a);   // current LPF
  m.adm_If += a * (m.I_meas - m.adm_If);
  if (m.adm_cal && (uint32_t)(millis() - m.adm_t0) > ADM_CAL_MS) {  // 1-shot gravity baseline
    m.adm_I0 = m.adm_If; m.adm_cal = false;
    Serial.printf("AK%u grav baseline I0=%.3f A captured (hold still done)\n", m.can_id, m.adm_I0);
  }
  float I = m.adm_If - m.adm_I0;                      // external current = total - gravity hold
  if (fabsf(I) < ADM_I_DB) I = 0.0f; else I -= (I > 0 ? ADM_I_DB : -ADM_I_DB); // deadband
  float tau_ext  = kt_tot * I;                        // external joint torque [N·m]
  float dx_tgt   = adm_sign * tau_ext / adm_Kv;       // compliance target offset [rad]
  float r = m.omega / ADM_OM_GATE;
  float gate = 1.0f / (1.0f + r * r);                 // ~1 quasi-static, ->0 when moving fast
  m.adm_dx += (gate * dx_tgt - m.adm_dx) / adm_tau * dt;  // gated lag, return tau = adm_tau
  m.adm_dx = clampf(m.adm_dx, -ADM_DX_MAX_DEG * DEG2RAD, ADM_DX_MAX_DEG * DEG2RAD);
  float x_cmd_deg = (m.adm_x0 + m.adm_dx) / DEG2RAD;
  motorSetPos(m.can_id, x_cmd_deg);
  m.theta_d = x_cmd_deg * DEG2RAD;                     // for telemetry
  m.dbg_I = I;
}

// ---- AK40 MIT native impedance: motor onboard does kp*(p_des-p)+kd*(0-v)+tff ---
//   ESP32 only streams the setpoint frame at 200Hz; stiffness/damping live in the
//   motor. kp[Nm/rad] kd[Nm/(rad/s)] at OUTPUT shaft. Motor mode must be entered
//   (0xFC) before enabling; exited (0xFD) on stop.
static void mit_step(AK40Motor &m, float dt) {
  if (!m.enabled || !m.mit_have) return;
  // vel+accel limited ramp of mit_pdes -> mit_pgoal (trapezoid): no position STEP,
  // so torque/current stay bounded. vcap = sqrt(2*a*dist) auto-decelerates to land.
  float perr = m.mit_pgoal - m.mit_pdes;
  float dir  = (perr >= 0.0f) ? 1.0f : -1.0f;
  float dist = fabsf(perr);
  float vcap = sqrtf(2.0f * mit_amax * dist);           // speed that still stops within dist
  if (vcap > mit_vmax) vcap = mit_vmax;                 // cruise cap -> trapezoid
  m.mit_pvel += clampf(dir * vcap - m.mit_pvel, -mit_amax * dt, mit_amax * dt);   // accel limit
  m.mit_pdes += m.mit_pvel * dt;
  if (dist < 0.5f * DEG2RAD && fabsf(m.mit_pvel) < 1.0f * DEG2RAD) {              // snap & settle
    m.mit_pdes = m.mit_pgoal; m.mit_pvel = 0.0f;
  }
  if (fabsf(m.mit_pdes - m.theta) > POS_ERR_LIMIT_DEG * DEG2RAD) {               // tracking-error e-stop
    m.enabled = false; mit_special(m.can_id, 0xFD);    // exit motor mode (limp)
    Serial.printf("!! AK%u MIT POS LIMIT -> EXIT\n", m.can_id);
    return;
  }
  float kd = m.mit_kd;                                  // hold damping
  if (m.mit_pvel != 0.0f && mit_kd_jog > kd) kd = mit_kd_jog;   // boost while ramping
  mit_pack(m.can_id, m.mit_pdes, 0.0f, m.mit_kp, kd, m.mit_tff);
}

// enter MIT motor mode, poll ~200ms with zero-gain frames to capture the live
// angle, then hold there. mit_pdes := theta (decoded from the SAME 16-bit P
// field) so there is no scale jump on entry. false if no MIT reply (not MIT fw).
static bool mit_enter_hold(AK40Motor &m) {
  if (!can_ok) { Serial.println("CAN not up"); return false; }
  mit_special(m.can_id, 0xFC);                 // enter motor mode
  m.mit_have = false;
  twai_message_t rx;
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    mit_pack(m.can_id, 0, 0, 0, 0, 0);
    unsigned long s = millis();
    while (millis() - s < 10) if (twai_receive(&rx, 0) == ESP_OK) parseMitReply(rx);
  }
  if (!m.mit_have) {
    mit_special(m.can_id, 0xFD);
    Serial.printf("AK%u: no MIT reply -> not MIT firmware? aborting hold\n", m.can_id);
    return false;
  }
  m.mit_pdes = m.mit_pgoal = m.theta; m.mit_pvel = 0.0f; m.enabled = true;   // hold at captured angle (no scale jump)
  Serial.printf(">> AK%u MIT HOLD at %.1f deg (kp=%.1f kd=%.2f tff=%.2f)\n",
                m.can_id, m.mit_pdes / DEG2RAD, m.mit_kp, m.mit_kd, m.mit_tff);
  return true;
}

// ---- AK40 inertia (J) test: J = kt_tot*(I2-I1)/(a2-a1), 2-level + rehome diff -
#define AK_J_I1   0.5f       // level-1 current [A] (gearbox stiction is high)
#define AK_J_MS   250        // pulse window [ms]
#define AK_J_TRAVEL_ABORT 1.2f   // [rad] stop pulse early if it runs away

static void can_pump() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) parseStatus(rx);
}
// constant-current pulse: drive m.dir*I_cmd for AK_J_MS, linear-fit omega(t) slope.
static bool ak_pulse(AK40Motor &m, float I_cmd, float &alpha, float &travel) {
  float th0 = m.theta;
  unsigned long t0 = micros(), tend = t0 + (unsigned long)AK_J_MS * 1000UL, lastS = 0;
  double n = 0, St = 0, Stt = 0, Sw = 0, Stw = 0;
  while (micros() < tend) {
    can_pump();
    motorSetCurrent(m.can_id, m.dir * I_cmd);
    if (fabsf(m.theta - th0) > AK_J_TRAVEL_ABORT) break;
    unsigned long now = micros();
    if (now - lastS >= 1000) {
      lastS = now;
      double t = (now - t0) * 1e-6, w = m.omega;
      n++; St += t; Stt += t * t; Sw += w; Stw += t * w;
    }
  }
  motorSetCurrent(m.can_id, 0.0f);
  double denom = n * Stt - St * St;
  alpha  = (denom != 0) ? (float)((n * Stw - St * Sw) / denom) : 0.0f;
  travel = m.theta - th0;
  return fabsf(alpha) > 0.5f;
}
static void ak_rehome(AK40Motor &m, float th_home, int ms) {
  m.theta_d = th_home; m.omega_f = 0; m.integ = 0; m.enabled = true; ak_eso_seed(m);
  unsigned long t = millis(), lc = micros();
  while (millis() - t < (unsigned long)ms) {
    can_pump();
    uint32_t now = micros();
    if ((uint32_t)(now - lc) >= AK40_CTRL_US) { float dt = (now - lc) * 1e-6f; lc = now; ak_holdStep(m, dt); }
  }
  m.enabled = false; motorSetCurrent(m.can_id, 0.0f);
}
void measure_inertia_ak(AK40Motor &m) {
  if (!can_ok || !m.haveStatus) { Serial.printf("AK%u: no status, J test refused\n", m.can_id); return; }
  float kt_tot = m.Kt * GEAR_RATIO * GEAR_EFF;
  float I1 = AK_J_I1, I2 = 2.0f * AK_J_I1, th_home = m.theta;
  Serial.printf("--- AK%u J test (2-level, %dms): I1=%.2f I2=%.2f A. MOTOR MOVES -- clear ---\n",
                m.can_id, AK_J_MS, I1, I2);
  float a1, tr1, a2, tr2;
  bool ok1 = ak_pulse(m, I1, a1, tr1);
  ak_rehome(m, th_home, 1200);
  bool ok2 = ak_pulse(m, I2, a2, tr2);
  ak_rehome(m, th_home, 1200);
  motorSetCurrent(m.can_id, 0.0f);
  Serial.printf("  L1: alpha=%.1f travel=%.3f | L2: alpha=%.1f travel=%.3f\n", a1, tr1, a2, tr2);
  if (!ok1 || !ok2) { Serial.println("  !! a pulse barely moved -- raise AK_J_I1"); return; }
  float dA = a2 - a1;
  if (fabsf(dA) < 1.0f) { Serial.println("  !! alpha2~alpha1 -- widen levels"); return; }
  float J = fabsf(kt_tot * (I2 - I1) / dA);
  Serial.printf("  >> J(AK%u joint) = %.3e kg·m^2  (kt_tot=%.3f). repeat 2-3x.\n", m.can_id, J, kt_tot);
}

// ============================================================================
// Encoder / gimbal motor init  (shared SPI bus)
// ============================================================================
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
  for (int i = 0; i < 5; i++) {
    m.initFOC();
    if (m.sensor_direction != Direction::UNKNOWN && m.zero_electric_angle > -1000.0f) {
      foc_zero = m.zero_electric_angle;
      Serial.printf("M%d FOC ok: zero=%.4f dir=%d\n", num, foc_zero, m.sensor_direction);
      return true;
    }
    Serial.printf("M%d initFOC FAIL retry %d/5\n", num, i + 1);
    delay(300);
  }
  Serial.printf("M%d FOC GAVE UP -- sensor dead\n", num);
  return false;
}

void setup_gimbal_motors() {
  // encoders init first -- they warm the shared bus before the IMU's first xfer
  encoder1.init(&SPI);
  encoder2.init(&SPI);
  motor1.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);

  driver1.voltage_power_supply = SUPPLY_VOLTAGE; driver1.voltage_limit = DRIVER_VLIMIT; driver1.init();
  driver2.voltage_power_supply = SUPPLY_VOLTAGE; driver2.voltage_limit = DRIVER_VLIMIT; driver2.init();
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
    m->voltage_sensor_align = 6.0f;   // raised 5->6: M2 load slip-stick during align
    m->controller = MotionControlType::torque;     // default: leveling (voltage PD)
    m->torque_controller = TorqueControlType::voltage;
    m->PID_velocity.P = 0.2f;                       // used by spin modes
    m->PID_velocity.I = 2.0f;
    m->PID_velocity.D = 0.0f;
    m->LPF_velocity.Tf = 0.01f;
  }

  m1_ok = init_one_motor(motor1, ENC1_CS, foc_zero1, 1);
  m2_ok = init_one_motor(motor2, ENC2_CS, foc_zero2, 2);
  motor1.disable();
  motor2.disable();
}

// Re-run current offset calibration with the motor bus powered (20V) + motors off.
// init() re-averages the idle ADC as zero; re-apply gain reversal after.
void recal_cs(InlineCurrentSense &cs, int num) {
  cs.init();
  cs.gain_a *= -1; cs.gain_b *= -1; cs.skip_align = true;
  PhaseCurrent_s c = cs.getPhaseCurrents();
  Serial.printf("  M%d recal -> rest Ia=%.3f Ib=%.3f (want ~0)\n", num, c.a, c.b);
}
void do_recal() {
  motor1.disable(); motor2.disable();
  Serial.println("--- recal current offsets (motors OFF, motor bus must be POWERED 20V) ---");
  recal_cs(cs1, 1);
  recal_cs(cs2, 2);
}

// ============================================================================
// IMU
// ============================================================================
bool setup_icm20948() {
  myICM.begin(IMU_CS, SPI, IMU_SPI_HZ);
  if (myICM.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fss; fss.a = gpm4; fss.g = dps500;
  myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_dlpcfg_t dlpcfg; dlpcfg.a = acc_d50bw4_n68bw8; dlpcfg.g = gyr_d51bw2_n73bw3;
  myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
  myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  ICM_20948_smplrt_t srd; srd.a = 4; srd.g = 4;   // 1125/(1+4)=225Hz > 100Hz poll -> always fresh
  myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
  myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);
  return (myICM.status == ICM_20948_Stat_Ok);
}

static inline float accel_phi_z(float ax, float ay) { return atan2f(-ax, -ay); }

void calibrate_gyro_bias() {
  Serial.println("Gyro bias calib -- KEEP STILL (2s)...");
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
  if (myICM.dataReady()) { myICM.getAGMT(); est_phi = accel_phi_z(myICM.accX(), myICM.accY()); }
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

// ============================================================================
// Current-sense AUTO verification (polarity-agnostic)
// ============================================================================
void do_current_auto(int idx) {
  BLDCDriver3PWM &drv = (idx == 0) ? driver1 : driver2;
  InlineCurrentSense &cs = (idx == 0) ? cs1 : cs2;
  const float Vt = 2.0f;
  Serial.printf("--- Current AUTO M%d (Vt=%.1f) ---\n", idx, Vt);
  drv.enable();
  drv.setPwm(0, 0, 0); delay(80);
  double oa = 0, ob = 0; for (int i = 0; i < 50; i++) { PhaseCurrent_s c = cs.getPhaseCurrents(); oa += c.a; ob += c.b; delayMicroseconds(200);} oa /= 50; ob /= 50;
  drv.setPwm(Vt, 0, 0); delay(150);
  double aa = 0, ab = 0; for (int i = 0; i < 50; i++) { PhaseCurrent_s c = cs.getPhaseCurrents(); aa += c.a; ab += c.b; delayMicroseconds(200);} aa /= 50; ab /= 50;
  drv.setPwm(0, 0, 0); drv.disable();
  Serial.printf("  offset Ia=%.3f Ib=%.3f | driveA Ia=%.3f Ib=%.3f\n", oa, ob, aa, ab);
  double da = aa - oa, db = ab - ob;
  bool pass = fabs(oa) < 0.15 && fabs(ob) < 0.15 && fabs(da) > 0.05 && fabs(da) > fabs(db);
  Serial.printf("  RESULT M%d: %s\n", idx, pass ? "PASS" : "FAIL -- check shunt/gain/wiring");
}

// ============================================================================
// Leveling hold (motor2 / IMU z)
// ============================================================================
void level_capture() {
  if (!imu_ok) { Serial.println("ERR: IMU not ok -- level-hold refused"); return; }
  if (!m1_ok)  { Serial.println("ERR: motor1 FOC failed -- level-hold refused"); return; }  // motor1 = pitch (good)
  target_phi = est_phi;
  enc1_at_C = encoder1.getAngle();
  motor1.controller = MotionControlType::torque;
  motor1.voltage_limit = MOTOR_VLIMIT;
  motor1.enable();
  holding = true;
  Serial.printf(">> LEVEL HOLD: target phi_z=%.1f deg (Kp=%.2f Kd=%.2f)\n",
                target_phi / DEG2RAD, Kp, Kd);
}

// ============================================================================
// Menu
// ============================================================================
void print_menu() {
  Serial.println("\n===== GimbalArm Integrated Self-Test (ESP32/D32) =====");
  Serial.printf (" IMU=%s CAN=%s M0(FOC)=%s M1(FOC)=%s  AK1=%s AK2=%s\n",
                 imu_ok?"OK":"FAIL", can_ok?"OK":"FAIL",
                 m1_ok?"OK":"FAIL", m2_ok?"OK":"FAIL",
                 ak[0].haveStatus?"OK":"---", ak[1].haveStatus?"OK":"---");
  Serial.println(" -- sensors --");
  Serial.println(" 2 IMU live     3 enc check    4 current raw");
  Serial.println(" 9 current AUTO('9'->'1'/'2')   c recal current(MKS 12V on)");
  Serial.println(" -- gimbal GM4108 --");
  Serial.println(" 5 spin M0      6 spin M1      l level-hold(M1/IMU z): K<Kp> D<Kd>");
  Serial.println(" -- AK40 QDD (CAN) --");
  Serial.println(" 7 AK40 status  8 AK40 hold('8'->'1'/'2')  o origin('o'->'1'/'2')  j J-test('j'->'1'/'2')");
  Serial.println("   ESO(in hold): B comp on/off  G<hz> bw  A<val> authority  E<val> J  I<val> Ki (sel id)");
  Serial.println("   tune(in hold): K<val> stiffness  D<val> damping (sel id)");
  Serial.println("   dither(in hold): Y<val> amp[A]  F<val> freq[Hz] (Y0=off)");
  Serial.println("   P SET_POS mode('P'->'1'/'2', T<deg> target) -- VESC onboard pos loop");
  Serial.println("   a admittance('a'->'1'/'2'): V<val> Kv[Nm/rad]  W<val> tau[s]  N flip sign");
  Serial.println("   m MIT-probe('m'->'1'/'2') -- test if firmware decodes MIT frames");
  Serial.println("   n MIT('n'->'1'/'2' single, '3'=BOTH) -- needs MIT firmware. H<Nm> ff");
  Serial.println("     single: K<stiff> D<damp> T<deg abs>   dual: T/t=AK1/AK2 jog, K/k=stiff, D/d=damp");
  Serial.println("     T/t ramped(vel+accel lim): S<deg/s>=speed R<deg/s2>=accel Z<val>=jog-damp");
  Serial.println(" x STOP -> menu   0/? this menu");
  Serial.println("======================================================");
}

void enter_mode(Mode m) {
  motor1.disable(); motor2.disable();
  holding = false;
  origin_pending = false;
  jtest_pending = false;
  mit_pending = false;
  mit_dual = false;
  for (int i = 0; i < AK40_NUM; i++) {
    ak[i].enabled = false; ak[i].mit_have = false;
    if (can_ok) { mit_special(ak[i].can_id, 0xFD); motorSetCurrent(ak[i].can_id, 0.0f); }  // exit MIT + zero servo
  }
  mode = m;
  switch (m) {
    case IMU_LIVE: Serial.println("--- IMU live (x=stop) ---"); break;
    case ENC_CHECK:
      Serial.printf("  ENC1(CS%d) %s   ENC2(CS%d) %s\n",
        ENC1_CS, encoder_alive(ENC1_CS) ? "ALIVE" : "NO RESP",
        ENC2_CS, encoder_alive(ENC2_CS) ? "ALIVE" : "NO RESP");
      break;
    case CURR_CHECK: Serial.println("--- current raw at rest (~0A expected, x=stop) ---"); break;
    case CURR_AUTO:  Serial.println("--- press '1'/'2' to inject & verify, x=menu ---"); break;
    case SPIN_M0:
      if (!m1_ok) { Serial.println("M0 FOC not OK"); mode = MENU; print_menu(); break; }
      Serial.printf("--- spin M0 %.1f rad/s closed (x=stop) ---\n", SPIN_VEL_RADS);
      motor1.controller = MotionControlType::velocity; motor1.voltage_limit = MOTOR_VLIMIT;
      motor1.enable(); motor1.target = SPIN_VEL_RADS; break;
    case SPIN_M1:
      if (!m2_ok) { Serial.println("M1 FOC not OK"); mode = MENU; print_menu(); break; }
      Serial.printf("--- spin M1 %.1f rad/s closed (x=stop) ---\n", SPIN_VEL_RADS);
      motor2.controller = MotionControlType::velocity; motor2.voltage_limit = MOTOR_VLIMIT;
      motor2.enable(); motor2.target = SPIN_VEL_RADS; break;
    case LEVEL_HOLD:
      Serial.println("--- LEVEL HOLD: hand-pose, motor2 holds phi_z. K<Kp> D<Kd> tune. x=stop ---");
      level_capture();
      break;
    case AK40_STATUS:
      Serial.println("--- AK40 status stream (x=stop) ---");
      Serial.println("  no status = no CAN reply. check TX5/RX33, transceiver, 120ohm, power");
      break;
    case AK40_HOLD:
      Serial.println("--- AK40 impedance hold: '1'/'2' pick id | B/G/A/E/I/K/D gains | x=stop ---");
      break;
    case AK40_POS:
      Serial.println("--- AK40 SET_POS (VESC onboard pos loop): '1'/'2' pick id, T<deg> target, x=stop ---");
      Serial.println("  NOTE: set VESC position PID gains in VESC Tool first, else won't hold");
      break;
    case AK40_ADM:
      Serial.println("--- AK40 ADMITTANCE: '1'/'2' pick id, V<val> Kv, W<val> tau(s), N flip, x=stop ---");
      Serial.println("  push joint -> it yields ~tau/Kv. small Kv=soft, big Kv=stiff. VESC pos PID must be set.");
      break;
    case AK40_MIT:
      Serial.println("--- AK40 MIT native stiffness: '1'/'2' pick id, '3'=BOTH | H<Nm ff> | x=stop ---");
      Serial.println("  needs MIT firmware (flashed via R-Link). motor does kp*(pdes-p)+kd*(-v)+tff onboard.");
      Serial.println("  single: K stiff, D damp, T<deg> abs target.");
      Serial.println("  '3'=BOTH: AK1<-T jog/K stiff/D damp | AK2<-t jog/k stiff/d damp (T/t rel +deg).");
      Serial.println("  T/t are vel+accel limited ramps (conservative) -- no step surge.");
      Serial.println("  S<deg/s> speed lim  R<deg/s2> accel lim  Z<val> jog-damp boost (while ramping).");
      break;
    default: break;
  }
}

void handle_key(char c) {
  if (c == '0' || c == '?') { mode = MENU; print_menu(); return; }
  if (c == 'x' || c == 'X') { Serial.println(">> stop"); enter_mode(MENU); print_menu(); return; }

  if (origin_pending && (c == '1' || c == '2')) {
    uint8_t id = (c == '2') ? M2_CAN_ID : M1_CAN_ID;
    motorSetOrigin(id);
    Serial.printf(">> AK%u origin set here\n", id);
    origin_pending = false;
    return;
  }
  if (jtest_pending && (c == '1' || c == '2')) {
    jtest_pending = false;
    enter_mode(MENU); measure_inertia_ak(ak[c == '2' ? 1 : 0]); print_menu();
    return;
  }
  if (mit_pending && (c == '1' || c == '2')) {
    mit_pending = false;
    enter_mode(MENU); mit_probe(c == '2' ? M2_CAN_ID : M1_CAN_ID); print_menu();
    return;
  }
  if (mode == CURR_AUTO && (c == '1' || c == '2')) {
    do_current_auto(c == '2' ? 1 : 0); enter_mode(MENU); print_menu(); return;
  }
  if (mode == AK40_HOLD && (c == '1' || c == '2')) {
    ak_sel = (c == '2') ? 1 : 0;
    AK40Motor &m = ak[ak_sel];
    if (!m.haveStatus) { Serial.printf("AK%u: no status yet, can't hold\n", m.can_id); return; }
    m.theta_d = m.theta; m.omega_f = 0; m.integ = 0; m.enabled = true; ak_eso_seed(m);
    Serial.printf(">> AK%u HOLD at %.1f deg (K=%.2f D=%.3f Ki=%.2f) ESO comp=%s wo=%.1fHz A=%.2f J=%.2e\n",
                  m.can_id, m.theta_d / DEG2RAD, m.K, m.D, m.Ki,
                  ak_dob_on ? "ON" : "OFF", ak_eso_wo / (2.0f * PI), ak_dob_scale, m.J_nom);
    return;
  }
  if (mode == AK40_POS && (c == '1' || c == '2')) {
    ak_sel = (c == '2') ? 1 : 0;
    AK40Motor &m = ak[ak_sel];
    if (!m.haveStatus) { Serial.printf("AK%u: no status yet\n", m.can_id); return; }
    m.theta_d = m.theta; m.enabled = true;
    motorSetPos(m.can_id, m.theta_d / DEG2RAD);
    Serial.printf(">> AK%u SET_POS hold at %.1f deg (should NOT jump; if it does, POS_SCALE wrong)\n",
                  m.can_id, m.theta_d / DEG2RAD);
    return;
  }
  if (mode == AK40_ADM && (c == '1' || c == '2')) {
    ak_sel = (c == '2') ? 1 : 0;
    AK40Motor &m = ak[ak_sel];
    if (!m.haveStatus) { Serial.printf("AK%u: no status yet\n", m.can_id); return; }
    m.adm_x0 = m.theta; m.theta_d = m.theta; m.enabled = true;   // anchor here
    m.adm_dx = 0.0f; m.adm_If = 0.0f; m.adm_I0 = 0.0f;           // reset admittance state
    m.adm_cal = true; m.adm_t0 = millis();                       // arm gravity-baseline calib
    motorSetPos(m.can_id, m.adm_x0 / DEG2RAD);
    Serial.printf(">> AK%u ADMITTANCE anchor=%.1f deg (Kv=%.2f tau=%.2fs sign=%+.0f). HOLD STILL ~0.4s (grav calib), then push.\n",
                  m.can_id, m.adm_x0 / DEG2RAD, adm_Kv, adm_tau, adm_sign);
    return;
  }
  if (mode == AK40_MIT && (c == '1' || c == '2')) {
    ak_sel = (c == '2') ? 1 : 0;
    mit_dual = false;
    mit_enter_hold(ak[ak_sel]);
    return;
  }
  if (mode == AK40_MIT && c == '3') {            // BOTH AK40 lock position simultaneously
    mit_dual = true;
    ak_sel = 0;
    for (int i = 0; i < AK40_NUM; i++) mit_enter_hold(ak[i]);
    Serial.println(">> DUAL MIT HOLD: AK1=T jog/K stiff/D damp | AK2=t jog/k stiff/d damp. x=stop");
    return;
  }

  switch (c) {
    case '2': enter_mode(IMU_LIVE); break;
    case '3': enter_mode(ENC_CHECK); break;
    case '4': enter_mode(CURR_CHECK); break;
    case '9': enter_mode(CURR_AUTO); break;
    case 'c': case 'C': do_recal(); mode = MENU; print_menu(); break;
    case '5': enter_mode(SPIN_M0); break;
    case '6': enter_mode(SPIN_M1); break;
    case 'l': case 'L': enter_mode(LEVEL_HOLD); break;
    case '7': enter_mode(AK40_STATUS); break;
    case '8': enter_mode(AK40_HOLD); break;
    case 'p': case 'P': enter_mode(AK40_POS); break;
    case 'a': enter_mode(AK40_ADM); break;   // lowercase only ('A' = ESO authority cmd)
    case 'n': enter_mode(AK40_MIT); break;   // lowercase only (MIT native stiffness)
    case 'j': case 'J':
      jtest_pending = true;
      Serial.println("J-test: press '1' or '2' to pick AK id (motor MOVES -- clear)"); break;
    case 'o': case 'O':
      origin_pending = true;
      Serial.println("set AK40 origin: press '1' or '2'"); break;
    case 'm': case 'M':
      mit_pending = true;
      Serial.println("MIT probe: press '1'/'2' to pick AK id (zero torque, safe)"); break;
    default: break;
  }
}

// AK40 ESO valued command: G<hz> bandwidth, A<val> authority, E<val> J for selected id
static void apply_eso_cmd(char cmd, const String &arg) {
  AK40Motor &m = ak[ak_sel];
  switch (cmd) {
    case 'G': if (arg.length()) set_ak_eso_bw(2.0f * PI * arg.toFloat());
              Serial.printf("ESO bw wo = %.2f Hz\n", ak_eso_wo / (2.0f * PI)); break;
    case 'A': if (arg.length()) ak_dob_scale = arg.toFloat();
              Serial.printf("ESO comp authority = %.2f (neg = flip sign)\n", ak_dob_scale); break;
    case 'I': if (arg.length()) { m.Ki = arg.toFloat(); m.integ = 0; }
              Serial.printf("AK%u Ki = %.3f (integ reset)\n", m.can_id, m.Ki); break;
    case 'K':
    case 'k': if (mode == AK40_MIT) {                      // MIT: K=AK1, k=AK2 stiffness (k=dual only)
                if (cmd == 'k' && !mit_dual) break;
                int idx = (cmd == 'k') ? 1 : (mit_dual ? 0 : ak_sel);
                if (arg.length()) ak[idx].mit_kp = arg.toFloat();
                Serial.printf("AK%u MIT kp = %.2f Nm/rad\n", ak[idx].can_id, ak[idx].mit_kp); break;
              }
              if (cmd == 'k') break;                         // lowercase only in MIT dual
              if (mode == LEVEL_HOLD) {                       // gimbal: motor2 leveling stiffness
                if (arg.length()) Kp = arg.toFloat();
                Serial.printf("gimbal M2 Kp = %.2f\n", Kp); break;
              }
              if (arg.length()) m.K = arg.toFloat();         // AK40 impedance hold (selected id)
              Serial.printf("AK%u K = %.3f\n", m.can_id, m.K); break;
    case 'D':
    case 'd': if (mode == AK40_MIT) {                      // MIT: D=AK1, d=AK2 damping (d=dual only)
                if (cmd == 'd' && !mit_dual) break;
                int idx = (cmd == 'd') ? 1 : (mit_dual ? 0 : ak_sel);
                if (arg.length()) ak[idx].mit_kd = arg.toFloat();
                Serial.printf("AK%u MIT kd = %.3f Nm/(rad/s)\n", ak[idx].can_id, ak[idx].mit_kd); break;
              }
              if (cmd == 'd') break;                         // lowercase only in MIT dual
              if (mode == LEVEL_HOLD) {                       // gimbal: motor2 leveling damping
                if (arg.length()) Kd = arg.toFloat();
                Serial.printf("gimbal M2 Kd = %.3f\n", Kd); break;
              }
              if (arg.length()) m.D = arg.toFloat();         // AK40 impedance hold (selected id)
              Serial.printf("AK%u D = %.4f\n", m.can_id, m.D); break;
    case 'H': if (arg.length()) m.mit_tff = arg.toFloat();
              Serial.printf("AK%u MIT tff = %.2f Nm\n", m.can_id, m.mit_tff); break;
    case 'E': if (arg.length()) m.J_nom = arg.toFloat();
              Serial.printf("AK%u ESO J = %.3e kg·m^2\n", m.can_id, m.J_nom); break;
    case 'T':
    case 't': if (mode == AK40_MIT && mit_dual) {          // dual: T=AK1, t=AK2 relative jog
                int idx = (cmd == 't') ? 1 : 0;
                if (arg.length()) {
                  ak[idx].mit_pgoal += arg.toFloat() * DEG2RAD;   // ramp target (vel/accel limited)
                  Serial.printf("MIT jog AK%u %+.1f deg -> goal %.1f deg\n",
                                ak[idx].can_id, arg.toFloat(), ak[idx].mit_pgoal / DEG2RAD);
                }
                break;
              }
              if (cmd == 't') break;                         // 't' only used in dual MIT hold
              if (mode == AK40_MIT) {                         // single MIT: absolute target on selected id
                if (arg.length()) m.mit_pgoal = arg.toFloat() * DEG2RAD;   // ramp target (vel/accel limited)
                Serial.printf("AK%u MIT goal = %.1f deg\n", m.can_id, m.mit_pgoal / DEG2RAD); break;
              }
              if (arg.length()) m.theta_d = arg.toFloat() * DEG2RAD;
              if (mode == AK40_POS && m.enabled) motorSetPos(m.can_id, m.theta_d / DEG2RAD);
              Serial.printf("AK%u target = %.1f deg\n", m.can_id, m.theta_d / DEG2RAD); break;
    case 'Y': if (arg.length()) ak_dither_amp = arg.toFloat();
              Serial.printf("dither amp = %.3f A\n", ak_dither_amp); break;
    case 'F': if (arg.length()) ak_dither_hz = arg.toFloat();
              Serial.printf("dither freq = %.1f Hz\n", ak_dither_hz); break;
    case 'V': if (arg.length()) adm_Kv = arg.toFloat();
              Serial.printf("admittance Kv = %.2f Nm/rad (%s)\n", adm_Kv,
                            adm_Kv < 1.0f ? "soft" : "stiff"); break;
    case 'W': if (arg.length()) adm_tau = arg.toFloat();
              Serial.printf("admittance tau = %.2f s (return time, Kv-independent)\n", adm_tau); break;
    case 'Z': if (arg.length()) mit_kd_jog = arg.toFloat();
              Serial.printf("MIT jog-damp boost = %.3f Nm/(rad/s) (while ramping)\n", mit_kd_jog); break;
    case 'S': if (arg.length()) mit_vmax = arg.toFloat() * DEG2RAD;
              Serial.printf("MIT jog speed limit = %.1f deg/s\n", mit_vmax / DEG2RAD); break;
    case 'R': if (arg.length()) mit_amax = arg.toFloat() * DEG2RAD;
              Serial.printf("MIT jog accel limit = %.1f deg/s^2\n", mit_amax / DEG2RAD); break;
  }
}

void process_serial() {
  static char pend = 0;            // valued ESO command awaiting numeric arg + delimiter
  static String num = "";
  while (Serial.available()) {
    char ch = Serial.read();
    if (pend) {
      if (ch == '\n' || ch == '\r' || ch == ' ') { apply_eso_cmd(pend, num); pend = 0; num = ""; }
      else num += ch;
      continue;
    }
    if (ch == 'G' || ch == 'A' || ch == 'E' || ch == 'I' || ch == 'K' || ch == 'k' || ch == 'D' || ch == 'd' || ch == 'T' || ch == 't' || ch == 'Y' || ch == 'F' || ch == 'V' || ch == 'W' || ch == 'H' || ch == 'Z' || ch == 'S' || ch == 'R') { pend = ch; num = ""; continue; }
    if (ch == 'B') {                // toggle ESO compensation (single char)
      ak_dob_on = !ak_dob_on;
      Serial.printf("ESO compensation %s\n", ak_dob_on ? "ON" : "OFF");
      continue;
    }
    if (ch == 'N') {                // flip admittance compliance sign (single char)
      adm_sign = -adm_sign;
      Serial.printf("admittance sign = %+.0f\n", adm_sign);
      continue;
    }
    if (ch == '\n' || ch == '\r' || ch == ' ') continue;
    handle_key(ch);
  }
}

// ============================================================================
// setup / loop
// ============================================================================
uint32_t lastAkCtrl = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n### GimbalArm Integrated Self-Test (ESP32/D32) booting ###");
  Serial.println("PSU 20V CURRENT LIMIT ~1.5A first. Motors twitch (FOC align). Keep still for gyro calib.");

  // shared SPI bus: bring up once, park all CS high (incl. unused IMU on bus)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
  pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);
  pinMode(IMU_CS,  OUTPUT); digitalWrite(IMU_CS,  HIGH);
  pinMode(22, INPUT);   // damaged GPIO22 (internal short to MOSI/23): never drive it

  Serial.print("Gimbal motors init... ");
  setup_gimbal_motors();
  Serial.println("done");

  Serial.print("IMU init... ");
  for (int i = 0; i < 10; i++) { if (setup_icm20948()) { imu_ok = true; break; } delay(200); }
  Serial.println(imu_ok ? "OK" : "FAILED (check IMU_CS=13, AD0=MISO, power)");
  if (imu_ok) calibrate_gyro_bias();

  ak[0] = AK40Motor{}; ak[0].can_id = M1_CAN_ID; ak[0].Kt = M1_KT; ak[0].dir = 1.0f; ak[0].K = K_IMP; ak[0].D = D_IMP; ak[0].J_nom = M1_J_NOM; ak[0].Ki = KI_IMP;
  ak[1] = AK40Motor{}; ak[1].can_id = M2_CAN_ID; ak[1].Kt = M2_KT; ak[1].dir = 1.0f; ak[1].K = K_IMP; ak[1].D = D_IMP; ak[1].J_nom = M2_J_NOM; ak[1].Ki = KI_IMP;
  for (int i = 0; i < AK40_NUM; i++) { ak[i].mit_kp = MIT_KP_DEF; ak[i].mit_kd = MIT_KD_DEF; ak[i].mit_tff = 0.0f; }
  set_ak_eso_bw(ak_eso_wo);   // compute ESO gains l1,l2,l3 from default bandwidth

  Serial.print("CAN(TWAI) init... ");
  can_ok = twai_setup();
  Serial.println(can_ok ? "OK (1Mbps)" : "FAILED (check SN65HVD230 wiring)");

  // AK40 retains last current cmd across ESP reset -> kill any leftover so it
  // doesn't spin at boot (enter_mode only zeros on first mode switch).
  if (can_ok) for (int i = 0; i < AK40_NUM; i++) {
    for (int r = 0; r < 5; r++) { motorSetCurrent(ak[i].can_id, 0.0f); delay(5); }
  }

  lastAkCtrl = micros();
  last_imu_us = micros();
  while (Serial.available()) Serial.read();   // flush boot glitch bytes (auto-reset RX noise)
  print_menu();
}

void loop() {
  // CAN RX
  if (can_ok) {
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) { parseStatus(rx); parseMitReply(rx); }
  }

  // gimbal FOC
  encoder1.update();
  if (m2_ok) encoder2.update();
  if (motor1.enabled) { motor1.loopFOC(); motor1.move(); }
  if (motor2.enabled) { motor2.loopFOC(); motor2.move(); }

  // AK40 200Hz impedance hold
  uint32_t now_us = micros();
  if ((uint32_t)(now_us - lastAkCtrl) >= AK40_CTRL_US) {
    float dt = (now_us - lastAkCtrl) * 1e-6f;
    lastAkCtrl = now_us;
    if (mode == AK40_HOLD) {
      ak_dither_ph += 2.0f * PI * ak_dither_hz * dt;                            // advance dither once per tick
      if (ak_dither_ph > 2.0f * PI) ak_dither_ph -= 2.0f * PI;
      for (int i = 0; i < AK40_NUM; i++) ak_holdStep(ak[i], dt);
    }
    else if (mode == AK40_STATUS)
      for (int i = 0; i < AK40_NUM; i++) motorSetCurrent(ak[i].can_id, 0.0f);  // poll status, 0 torque
    else if (mode == AK40_POS) {
      for (int i = 0; i < AK40_NUM; i++) {
        if (ak[i].enabled) motorSetPos(ak[i].can_id, ak[i].theta_d / DEG2RAD);  // keep-alive VESC pos loop
        else motorSetCurrent(ak[i].can_id, 0.0f);                               // poll status (fresh theta) pre-hold
      }
    }
    else if (mode == AK40_ADM) {
      for (int i = 0; i < AK40_NUM; i++) {
        if (ak[i].enabled) ak_admStep(ak[i], dt);                               // SET_POS + current yield
        else motorSetCurrent(ak[i].can_id, 0.0f);                              // poll status pre-anchor
      }
    }
    else if (mode == AK40_MIT) {
      for (int i = 0; i < AK40_NUM; i++)
        if (ak[i].enabled) mit_step(ak[i], dt);                                 // native impedance setpoint frame
    }
  }

  // IMU 100Hz attitude + leveling PD
  if (imu_ok && (uint32_t)(now_us - last_imu_us) >= IMU_PERIOD_US && myICM.dataReady()) {
    float dt = (now_us - last_imu_us) * 1e-6f;
    last_imu_us = now_us;
    float gz_rads = 0;
    update_attitude(dt, gz_rads);
    if (mode == LEVEL_HOLD && holding) {
      if (fabsf(encoder1.getAngle() - enc1_at_C) > SOFT_LIMIT_RAD) {
        Serial.println("!! SOFT LIMIT -- emergency stop");
        enter_mode(MENU); print_menu();
      } else {
        float e = target_phi - est_phi;
        float v1 = SIGN_X * (Kp * e - Kd * gz_rads);
        motor1.target = constrain(v1, -MOTOR_VLIMIT, MOTOR_VLIMIT);
      }
    }
  }

  process_serial();

  // 5Hz reporting
  if (millis() - last_print_ms >= 200) {
    last_print_ms = millis();
    switch (mode) {
      case IMU_LIVE:
        if (myICM.dataReady()) {
          myICM.getAGMT();
          Serial.printf("  acc[mg] %.0f %.0f %.0f | gyr[dps] %.1f %.1f %.1f | phi_z=%.1f deg\n",
            myICM.accX(), myICM.accY(), myICM.accZ(),
            myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(), est_phi / DEG2RAD);
        } else Serial.println("  IMU no data");
        break;
      case ENC_CHECK:
        Serial.printf("  enc1=%.3f rad  enc2=%.3f rad\n", encoder1.getAngle(), encoder2.getAngle());
        break;
      case CURR_CHECK: {
        PhaseCurrent_s a = cs1.getPhaseCurrents();
        PhaseCurrent_s b = cs2.getPhaseCurrents();
        Serial.printf("  M0 Ia=%.3f Ib=%.3f | M1 Ia=%.3f Ib=%.3f [A]\n", a.a, a.b, b.a, b.b);
        break;
      }
      case SPIN_M0:
        Serial.printf("  M0 vel=%.2f (tgt %.1f) ang=%.2f\n",
          motor1.shaft_velocity, SPIN_VEL_RADS, encoder1.getAngle());
        break;
      case SPIN_M1:
        Serial.printf("  M1 vel=%.2f (tgt %.1f) ang=%.2f\n",
          motor2.shaft_velocity, SPIN_VEL_RADS, encoder2.getAngle());
        break;
      case LEVEL_HOLD:
        Serial.printf("  phi=%.1f tgt=%.1f deg | v1=%.2f | hold=%d\n",
          est_phi / DEG2RAD, target_phi / DEG2RAD, motor2.target, holding);
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
            Serial.printf("  AK%u hold th=%.1f tgt=%.1f I=%.2f A | taui=%.3f d_hat=%.4f w_eso=%.2f comp=%s\n",
              ak[i].can_id, ak[i].theta / DEG2RAD, ak[i].theta_d / DEG2RAD, ak[i].dbg_I,
              ak[i].dbg_taui, ak[i].d_hat, ak[i].eso_x2, ak_dob_on ? "ON" : "OFF");
        break;
      case AK40_POS:
        if (ak[ak_sel].enabled)
          Serial.printf("  AK%u SET_POS th=%.1f tgt=%.1f deg (err=%.1f)\n",
            ak[ak_sel].can_id, ak[ak_sel].theta / DEG2RAD, ak[ak_sel].theta_d / DEG2RAD,
            (ak[ak_sel].theta_d - ak[ak_sel].theta) / DEG2RAD);
        break;
      case AK40_ADM:
        if (ak[ak_sel].enabled)
          Serial.printf("  AK%u ADM x0=%.1f th=%.1f cmd=%.1f | I_ext=%.2fA w=%.2f dx=%.1fdeg Kv=%.2f tau=%.2f\n",
            ak[ak_sel].can_id, ak[ak_sel].adm_x0 / DEG2RAD, ak[ak_sel].theta / DEG2RAD,
            ak[ak_sel].theta_d / DEG2RAD, ak[ak_sel].dbg_I, ak[ak_sel].omega,
            ak[ak_sel].adm_dx / DEG2RAD, adm_Kv, adm_tau);
        break;
      case AK40_MIT:
        if (mit_dual) {
          for (int i = 0; i < AK40_NUM; i++)
            if (ak[i].enabled)
              Serial.printf("  AK%u MIT th=%.1f goal=%.1f deg | tau=%.2fNm w=%.2f kd=%s\n",
                ak[i].can_id, ak[i].theta / DEG2RAD, ak[i].mit_pgoal / DEG2RAD,
                ak[i].mit_tau, ak[i].omega, ak[i].mit_pvel != 0.0f ? "JOG" : "hold");
        } else if (ak[ak_sel].enabled)
          Serial.printf("  AK%u MIT th=%.1f goal=%.1f deg | tau=%.2fNm w=%.2f | kp=%.1f kd=%.2f tff=%.2f\n",
            ak[ak_sel].can_id, ak[ak_sel].theta / DEG2RAD, ak[ak_sel].mit_pgoal / DEG2RAD,
            ak[ak_sel].mit_tau, ak[ak_sel].omega,
            ak[ak_sel].mit_kp, ak[ak_sel].mit_kd, ak[ak_sel].mit_tff);
        break;
      default: break;
    }
  }
}
