/**
 * arm_microros_esp32.ino  -  LOLIN D32 (ESP32)
 * GimbalArm micro-ROS joint controller -- D2 (real AK40 MIT control).
 *
 * Subscribes  /joint_commands (sensor_msgs/JointState, rad, JOINT_NAMES order)
 *             /home           (std_msgs/Empty)  -> capture current pose as zero
 *             /arm_enable     (std_msgs/Bool)   -> energize / limp the AK40s
 * Publishes   /joint_states   (sensor_msgs/JointState, rad)
 * Joints (order): base_yaw, shoulder, elbow, wrist_pitch, wrist_roll   domain=5
 *
 * HARDWARE REALITY (2026-06-16): base_yaw servo ABSENT, wrist_roll gimbal DEAD
 * (one phase open).  Only the two AK40 QDDs (shoulder, elbow) and the wrist_pitch
 * gimbal are usable.  The arm is therefore planar (all live joints rotate about
 * Y) -> wrist_pitch alone keeps the cup level; wrist_roll is not needed.
 *   shoulder -> ak[0] (CAN id 1, MIT) | elbow -> ak[1] (CAN id 2, MIT)
 *   wrist_pitch -> autonomous IMU level-hold (GM4108 SimpleFOC + ICM20948, PD).
 *                  /level_enable Bool starts/stops it; target = world level.
 *   base_yaw / wrist_roll -> reported 0, never driven.
 *
 * HOMING (SW offset, stored in ESP32 NVS):  q = dir*(theta_raw - offset).
 *   1) boot -> AK40 enter MIT motor mode, ZERO-gain frames = backdrivable, theta live
 *   2) pull arm straight UP (vertical = URDF home, q_sh = q_el = 0)
 *   3) publish /home -> offset[i] = theta_raw[i], saved to NVS
 *   4) publish /arm_enable true -> hold at q=0 (kp ramps in motor)
 *   NOTE: AK40-10 = rotor abs encoder + 10:1 gear -> OUTPUT angle is NOT
 *   power-cycle absolute (multiturn ambiguity).  The NVS offset only removes
 *   re-homing IF theta_raw at a fixed pose repeats across power cycles -- TEST
 *   that first (cycle power 3x, read theta at same pose).  If it jumps, re-home
 *   each boot (one /home publish).  VESC SET_ORIGIN_HERE is useless here: the
 *   AK40 runs MIT firmware (standard 11-bit frames only), not VESC.
 *
 * KEY FIX (kept from D1): non-blocking micro-ROS ping/reconnect state machine, so
 * the agent can be (re)started after the MCU boots -- NO replug.
 *
 * micro-ROS transport = USB serial @115200 (ESP32 = /dev/ttyUSB*).
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
 * NOTE: serial is owned by micro-ROS -- no Serial.print debug here.
 * NOTE: GPIO5 = CAN TX (to SN65HVD230) = also the D32 onboard LED pin, so there is
 *   NO status LED in this build (CAN owns the pin).
 *
 * Libraries: micro_ros_arduino, ESP32 core (driver/twai.h), Preferences
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>
#include "driver/twai.h"
#include <Preferences.h>
#include <math.h>
#include <SimpleFOC.h>          // wrist_pitch GM4108 FOC (leveling)
#include <SPI.h>                // shared bus: AS5048A encoder + ICM20948 IMU
#include <ICM_20948.h>          // wrist IMU (SparkFun)

// ─── config ──────────────────────────────────────────────────────────────────
#define ROS_DOMAIN_ID  5
#define NUM_JOINTS     5
#define STATE_HZ       25                        // /joint_states publish rate. 50Hz x ~240B
                                                  // = 12KB/s > 115200 serial cap -> TX backed up,
                                                  // publish blocked, FOC starved.  25Hz fits.
#define STATE_PERIOD_MS (1000 / STATE_HZ)
#define CMD_WATCHDOG_MS 500                       // no command -> hold last goal
#define DRY_RUN        0                          // 1 = no CAN, echo only (ROS plumbing test)
#define WRIST_ENABLE   1                          // 0 = skip wrist FOC/IMU ticks (isolation). 1 = run leveling
#define WR2_ENABLE     1                          // wrist_roll(motor2): roll motor REPLACED 2026-06-24 -> driven.
                                                  // enc2 on CS15 (off dead GPIO22). WR_SIGN2/phi_x mapping still HW-unverified.

// CAN / TWAI  (to SN65HVD230 -> AK40 ids 1,2, 120ohm both ends)
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_33

#define AK40_CTRL_HZ 200
#define AK40_CTRL_US (1000000UL / AK40_CTRL_HZ)
#define DEG2RAD  0.0174532925f

// joint <-> motor map
#define SH_JIDX  1                                // shoulder = JOINT_NAMES[1] -> ak[0]
#define EL_JIDX  2                                // elbow    = JOINT_NAMES[2] -> ak[1]
#define WP_JIDX  3                                // wrist_pitch (leveling placeholder)
#define AK40_NUM 2
// physical motor swap 2026-06-17 (shoulder motor had a stripped fastener -> moved to
// the lower-load elbow mount).  ch0=shoulder now talks to id 2, ch1=elbow to id 1.
static const uint8_t AK40_CAN_ID[AK40_NUM] = {2, 1};
// dir maps motor theta sign -> joint +.  Verified on HW 2026-06-17 by backdrivable
// hand-move vs RViz (feedback-only, so dir shows directly): shoulder correct, elbow
// reversed -> flipped to -1.  See ros2-stack D2 "wrist 부호검증".
static float AK40_DIR[AK40_NUM] = {+1.0f, -1.0f};

// boot home-recovery (sector snap): AK40-10 loses output multiturn count on power-down
// -> theta_raw at a fixed pose jumps by one gear sector (2*pi/gear rad) across boots,
// but is exact WITHIN a sector.  On boot we snap the stored NVS home offset to the
// sector nearest the current reading -> recovers home with NO re-home, IF the arm sits
// within +-half a sector (~18 deg output) of home when the agent connects.
// CALIBRATE: if the observed boot jump != ARM_SECTOR_RAD, set it to the measured jump.
#define ARM_GEAR        10.0f
#define ARM_SECTOR_RAD  (6.2831853f / ARM_GEAR)   // 0.6283 rad = 36 deg output

// MIT native impedance gains (motor onboard loop). Soft start; gravity tff = next
// tuning step (project_ak40_control_tuning), tff=0 for now -> expect slight sag.
#define MIT_KP_HOLD 36.0f      // [Nm/rad]  (3x; range 0..MIT_KP_MAX=500)
#define MIT_KD_HOLD 0.6f       // [Nm/(rad/s)]
#define MIT_TFF     0.0f       // [Nm] feed-forward (gravity comp TODO)
#define KP_SLEW     90.0f      // [Nm/rad/s] kp soft-start slew on enable (0->36 ~0.4s)
// jog profile caps (mirror hw_selftest proven values)
static const float MIT_VMAX   = 20.0f * DEG2RAD;   // [rad/s]
static const float MIT_AMAX   = 40.0f * DEG2RAD;   // [rad/s^2]
static const float MIT_KD_JOG = 0.1f;              // damping boost while ramping
#define POS_ERR_LIMIT_DEG 90.0f                    // tracking-error e-stop (debounced)
#define ESTOP_DEBOUNCE    5                         // consecutive over-limit ticks -> trip (~25ms): ignore 1-frame CAN/theta glitches
#define TEMP_WARN         176                       // [reply temp units = degF, suspected] warn ~80degC (176degF)
#define TEMP_LIMP         194                       // [reply temp units = degF, suspected] limp ~90degC (194degF). byte6 reads ~115 cold -> treating as Fahrenheit. Motor onboard hard-trip is backstop.
#define THERMAL_KP_RATE   1.0f                      // kp_scale ramp-down per sec on thermal -> gentle sag, no floor slam

// MIT frame ranges (T-Motor standard)
#define MIT_P_MIN -12.5f
#define MIT_P_MAX  12.5f
#define MIT_V_MIN -65.0f
#define MIT_V_MAX  65.0f
#define MIT_KP_MAX 500.0f
#define MIT_KD_MAX 5.0f
#define MIT_T_MIN -18.0f
#define MIT_T_MAX  18.0f

// ─── wrist_pitch leveling (GM4108 via SimpleFOC + ICM20948 IMU) ──────────────
// Ported from gimbal_level_hold_esp32 (PD-only, no ESO).  Autonomous ABSOLUTE
// level: target tilt = 0 (world level), IMU mounted -y up.  Independent of the
// AK40 arm command -- keeps the cup horizontal regardless of shoulder/elbow pose.
// Same ESP32, shared SPI bus; CAN (5,33) does not collide with these pins.
#define WR_PWM_A 25
#define WR_PWM_B 26
#define WR_PWM_C 27
#define WR_EN    14
#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define WR_ENC_CS 21
#define WR_ENC2_CS 15   // enc2 CS moved off GPIO22 (D32 has an internal GPIO22<->23/MOSI short
                        // that corrupted the whole SPI bus).  GPIO22 left UNDRIVEN (input) below.
#define WR_IMU_CS 13
#define GM4108_PP        11
#define GM4108_PHASE_R   5.5f
#define WR_SUPPLY_V      20.0f   // verified gimbal supply (project_hw_next_steps)
#define WR_DRIVER_VLIMIT 18.0f   // raised 12->18 (supply 20V) so motor VLIMIT isn't capped by driver
#define WR_VLIMIT        16.0f   // raised 10->16: 16/5.5~=2.9A peak ~0.45Nm (was 0.28). ⚠️thermal: short bursts OK, watch heat
#define WR_ILIMIT        5.0f    // raised 2->5: current limit released for tuning. ⚠️GM4108 cont. current low
#define WR_VEL_LIMIT     20.0f
#define WR_ENC_SPI_HZ    1000000UL   // back to 1M (the bus failure was the D32 GPIO22<->23 short,
#define WR_IMU_SPI_HZ    1000000UL   // not signal integrity; fixed by remapping ENC2_CS off 22)
#define WR_IMU_HZ        300     // raised 100->300: cut PD phase lag (heavy roll felt laggy). ODR must exceed this (srd=2 -> 375Hz). prev "300Hz for bandwidth"
                                 // change was chasing the GPIO22-23 short's MISO-noise divergence, not real)
#define WR_IMU_PERIOD_US (1000000UL / WR_IMU_HZ)
#define WR_COMP_ALPHA    0.98f   // hw_selftest verified complementary weight @100Hz
#define WR_SIGN          (-1.0f) // motor1 phi_z sign (hw_selftest verified). flip if it diverges
#define WR_ATT_LIMIT     0.61f   // |est_phi| e-stop [rad ~35deg] = leveling diverged / cup tipping

// initFOC alignment skip: hardcode zero_electric_angle+direction -> NO boot rotation.
// Leave at NO_CAL to run alignment once (it prints "* cal: zero=.. dir=.." -> copy here).
// AS5048A is absolute so the zero is stable across reboots (re-cal only if motor/encoder remounted).
#define NO_CAL        1000.0f
#define WR_CAL_ZERO   3.6877f    // motor1 (pitch) zero_electric_angle [rad] -- measured 2026-06-24 unloaded
#define WR_CAL_DIR    (-1)       // 1=CW  -1=CCW  (used only when *_CAL_ZERO != NO_CAL)
#define WR2_CAL_ZERO  3.5266f    // motor2 (roll)  -- measured 2026-06-24 unloaded
#define WR2_CAL_DIR   (-1)
float wrist_kp = 10.0f;          // [V/rad] motor1(pitch). HW-tuned 2026-06-24 (stable w/ DOB). runtime /wrist_cfg
float wrist_kd = 0.25f;          // [V/(rad/s)]
float wrist_kp2 = 40.0f;         // [V/rad] motor2(roll) -- heavier axis (pitch+cup inertia), needs higher kp
float wrist_kd2 = 0.49f;         // [V/(rad/s)]  via /wrist_cfg[6],[7]

// ESO disturbance observer (Level3 DOB) -- the project's core goal.  Ported from
// gimbal_level_hold (same GM4108, J/Kt/R below are its measured/verified values).
//   plant w_dot = b0*i + f, b0=Kt/J, f=d/J.  states [theta,omega,f], Gao triple
//   pole at -wo: l1=3wo, l2=3wo^2, l3=wo^3.  d_hat = J*x3.  comp voltage added to PD.
// Needs phase-current sense (Iq) -> InlineCurrentSense on the wrist driver shunts.
#define WR_CS_A      36          // current-sense ADC A (motor1 ch, gimbal CS1_A)
#define WR_CS_B      39          // current-sense ADC B (gimbal CS1_B)
#define WR_CS_SHUNT  0.01f
#define WR_CS_GAIN   50.0f
#define WR_J_MEAS    6.8e-5f     // GM4108 inertia [kg·m^2] (2026-06-13)
#define WR_DOB_R     5.5f        // phase R [ohm], torque->voltage map
#define WR_DOB_KT    0.154f      // GM4108 Kt [N·m/A]
#define WR_DOB_VCLAMP 3.0f       // |comp voltage| clamp [V]
#define WR_IQ_TAU    0.008f      // ~20Hz Iq filter
#define WR_V_TAU     0.006f      // ~27Hz output-voltage LPF (kills audible buzz; leveling <5Hz)
float wrist_dob_scale = -0.43f;  // comp authority (signed). NEG = disturbance rejection (DOB textbook).
                                 // HW 2026-06-24: + felt like compliance (assists disturbance), - rejects. -0.43 stable.
bool  wrist_dob_on    = true;    // DOB compensation enable (project goal -> default ON)

// ─── 2nd gimbal axis: wrist_roll (motor2, GM4108) -- 2-axis cup leveling ──────
// Mirrors motor1.  motor2 corrects the OTHER tilt (IMU x-axis) so the cup stays
// level in both directions.  Shares the /wrist_cfg gains; separate sign + ESO state.
// ⚠️ UNVERIFIED on HW (no IMU/motor2 yet): WR_SIGN2 and the phi_x axis mapping must
//    be checked when parts arrive (flip WR_SIGN2 / fix phi_x if it diverges).
#define WR2_PWM_A 16             // gimbal M2_PWM_A
#define WR2_PWM_B 17
#define WR2_PWM_C 4
#define WR2_EN    12
#define WR2_ENC_CS WR_ENC2_CS    // =15. enc2 CS remapped off dead GPIO22 (D32 internal 22<->23/MOSI short). Was 22 = dead pin -> enc2 no-resp -> motor2 FOC init skipped -> roll didn't turn.
#define WR2_CS_A   34            // current-sense ADC (gimbal CS2_A)
#define WR2_CS_B   32            // gimbal CS2_B
#define WR_SIGN2   (+1.0f)       // motor2 phi_x sign -- flipped -1->+1 (roll diverged on HW 2026-06-24)
#define WR_GYRO2_SIGN (-1.0f)    // roll gyro gx sign: accel_phi2 uses atan2(-az,-ay) (z leads) which is
                                 // opposite handedness to gx integration -> est_phi2 fought accel (lag) and
                                 // -kd*gx was anti-damping (kd worsened divergence).  Flip gx to fix both.

static const char *JOINT_NAMES[NUM_JOINTS] = {
    "base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"};

// joint limits [rad] -- mirror gimbal_arm_controller/joint_config.py JOINT_LIMITS
static const float JLIM_LO[NUM_JOINTS] = {-1.5708f, -1.5708f, -2.0944f, -0.7854f, -0.7854f};
static const float JLIM_HI[NUM_JOINTS] = { 1.5708f,  1.5708f,  2.0944f,  0.7854f,  0.7854f};

// ─── micro-ROS handles ───────────────────────────────────────────────────────
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;
rcl_publisher_t    state_pub;
// NOTE: micro_ros_arduino prebuilt limits RMW_UXRCE_MAX_SUBSCRIPTIONS = 5.
// We use exactly 5: cmd, home, enable, level, cfg.  /set_kp was dropped to make
// room for wrist leveling (MIT kp stays at the compile default MIT_KP_HOLD).
// /wrist_cfg (Float32MultiArray) carries [kp, kd, dob_scale, dob_bw_hz, dob_on]
// so all wrist + DOB tuning is live without burning extra subscription slots.
rcl_subscription_t cmd_sub, home_sub, enable_sub, level_sub, cfg_sub;
rclc_executor_t    executor;
sensor_msgs__msg__JointState state_msg;
sensor_msgs__msg__JointState cmd_msg;
std_msgs__msg__Empty         home_msg;
std_msgs__msg__Bool          enable_msg;
std_msgs__msg__Bool          level_msg;
std_msgs__msg__Float32MultiArray cfg_msg;

Preferences prefs;

// ─── wrist_pitch leveling state (GM4108 + ICM20948) ──────────────────────────
BLDCMotor      wrist_motor  = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM wrist_drv    = BLDCDriver3PWM(WR_PWM_A, WR_PWM_B, WR_PWM_C, WR_EN);
MagneticSensorSPIConfig_s WR_AS5048_CFG = {
  .spi_mode = SPI_MODE1, .clock_speed = WR_ENC_SPI_HZ, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI wrist_enc = MagneticSensorSPI(WR_AS5048_CFG, WR_ENC_CS);
InlineCurrentSense wrist_cs = InlineCurrentSense(WR_CS_SHUNT, WR_CS_GAIN, WR_CS_A, WR_CS_B);
ICM_20948_SPI     wrist_imu;
bool  wrist_ok      = false;     // FOC init ok
bool  wrist_imu_ok  = false;     // IMU init ok
bool  wrist_hold    = false;     // leveling active (motor enabled)
float wrist_est_phi = 0.0f;      // estimated cup tilt [rad], 0 = level
float wrist_gbias_z = 0.0f;      // gyro z bias [rad/s]
float wrist_vpd     = 0.0f;      // PD output voltage applied in FOC tick
float wrist_enc0    = 0.0f;      // encoder angle at level-enable (joint q=0 ref)
// ESO state (states [x1=theta, x2=omega, x3=f=d/J]); d_hat = J*x3
float wr_eso_x1 = 0, wr_eso_x2 = 0, wr_eso_x3 = 0, wr_dhat = 0;
float wr_eso_wo = 2.0f * PI * 6.78f;          // observer bw [rad/s] HW-tuned 6.78Hz (<9Hz mech resonance)
float wr_eso_l1 = 0, wr_eso_l2 = 0, wr_eso_l3 = 0;
float wrist_J   = WR_J_MEAS;                   // runtime inertia (bare vs +cup) via /wrist_cfg[5]
float wr_eso_b0 = WR_DOB_KT / WR_J_MEAS;       // input gain Kt/J (recomputed when wrist_J changes)
float wr_iq_filt = 0.0f;
unsigned long wr_last_iq_us = 0;
unsigned long wrist_last_imu_us = 0;

// ─── 2nd axis (wrist_roll / motor2) objects + state (mirror of motor1) ────────
BLDCMotor      wrist_motor2 = BLDCMotor(GM4108_PP, GM4108_PHASE_R);
BLDCDriver3PWM wrist_drv2   = BLDCDriver3PWM(WR2_PWM_A, WR2_PWM_B, WR2_PWM_C, WR2_EN);
MagneticSensorSPI wrist_enc2 = MagneticSensorSPI(WR_AS5048_CFG, WR2_ENC_CS);
InlineCurrentSense wrist_cs2 = InlineCurrentSense(WR_CS_SHUNT, WR_CS_GAIN, WR2_CS_A, WR2_CS_B);
bool  wrist_ok2      = false;    // motor2 FOC init ok
float wrist_est_phi2 = 0.0f;     // estimated cup tilt about IMU x [rad]
float wrist_gbias_x  = 0.0f;     // gyro x bias [rad/s]
float wrist_vpd2     = 0.0f;
float wrist_enc0_2   = 0.0f;
float wr2_eso_x1 = 0, wr2_eso_x2 = 0, wr2_eso_x3 = 0, wr2_dhat = 0;
float wr2_iq_filt = 0.0f;
unsigned long wr2_last_iq_us = 0;
// motor2 shares wr_eso_wo/l1..l3/wrist_J/wr_eso_b0 and the /wrist_cfg gains with motor1

// ─── AK40 state ──────────────────────────────────────────────────────────────
struct AK40Motor {
  uint8_t can_id; float dir;
  float theta, omega, mit_tau;  bool mit_have;     // decoded from CAN reply
  uint8_t mtemp, merr;                             // motor temp[C], error code (reply bytes 6,7)
  float kp_scale;                                  // thermal derate 1->0 (gentle limp on overtemp)
  bool  thermal;                                   // thermal limp engaged
  uint8_t estop_cnt;                               // e-stop debounce counter
  float offset;                                    // theta_raw at home (q=0)
  float mit_pgoal, mit_pdes, mit_pvel;             // goal / ramped setpoint / profile vel [rad, motor frame]
  bool  enabled;                                   // true = active gains, false = limp/backdrivable
};
AK40Motor ak[AK40_NUM];

float g_mit_kp  = MIT_KP_HOLD;  // runtime MIT stiffness target [Nm/rad], set via /set_kp
float g_kp_ramp = 0.0f;         // soft-start ramped stiffness actually streamed to motors

float          g_cmd[NUM_JOINTS]   = {0, 0, 0, 0, 0};   // latest commanded joint pos (clamped) [rad]
float          g_state[NUM_JOINTS] = {0, 0, 0, 0, 0};   // reported joint pos [rad]
unsigned long  last_cmd_ms   = 0;
unsigned long  last_state_ms = 0;
unsigned long  last_ak_us    = 0;
bool           can_ok        = false;
bool           can_entered   = false;                   // AK40 in MIT motor mode
bool           offset_snapped = false;                  // boot sector-snap done once

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile unsigned long _last = 0; \
    if (millis() - _last > (unsigned long)(MS)) { X; _last = millis(); } \
  } while (0)

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ─── joint <-> motor frame ───────────────────────────────────────────────────
static inline float q_from_theta(int k, float theta) { return ak[k].dir * (theta - ak[k].offset); }
static inline float theta_from_q(int k, float q)     { return ak[k].offset + ak[k].dir * q; }

// ─── park wired driver pins (brownout guard) ─────────────────────────────────
// Both gimbal motors (motor1 25/26/27/14, motor2 16/17/4/12) are now SimpleFOC-owned
// (claimed in setup_wrist), so there are no unused floating gimbal pins to park.
// On disconnect we DISABLE the motors (wrist_level_stop) instead of yanking pins.
// CAN pins (5,33) = TWAI-owned.  No-op kept for call-site compatibility.
static void park_gimbal() { /* no pins to park (all motor pins SimpleFOC-driven) */ }

// ─── MIT primitives (verbatim from hw_selftest_esp32, proven) ────────────────
static uint16_t float_to_uint(float x, float lo, float hi, int bits) {
  if (x < lo) x = lo; else if (x > hi) x = hi;
  return (uint16_t)((x - lo) * (float)((1u << bits) - 1u) / (hi - lo));
}
static float uint_to_float(uint16_t x, float lo, float hi, int bits) {
  return lo + (float)x * (hi - lo) / (float)((1u << bits) - 1u);
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
  b[0] = pi >> 8;            b[1] = pi & 0xFF;
  b[2] = vi >> 4;           b[3] = ((vi & 0xF) << 4) | (ki >> 8);
  b[4] = ki & 0xFF;         b[5] = di >> 4;
  b[6] = ((di & 0xF) << 4) | (ti >> 8);   b[7] = ti & 0xFF;
  canSendStd(id, b, 8);
}
// MIT reply: b0=id, b1..2=pos16, b3=vel hi8, b4=(vel lo4)<<4|(cur hi4), b5=cur lo8.
static void parseMitReply(const twai_message_t &msg) {
  if (msg.extd || msg.data_length_code < 6) return;
  uint8_t id = msg.data[0];
  for (int k = 0; k < AK40_NUM; k++) {
    if (ak[k].can_id != id) continue;
    uint16_t pi = ((uint16_t)msg.data[1] << 8) | msg.data[2];
    uint16_t vi = ((uint16_t)msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t ci = (((uint16_t)(msg.data[4] & 0xF)) << 8) | msg.data[5];
    ak[k].theta   = uint_to_float(pi, MIT_P_MIN, MIT_P_MAX, 16);
    ak[k].omega   = uint_to_float(vi, MIT_V_MIN, MIT_V_MAX, 12);
    ak[k].mit_tau = uint_to_float(ci, MIT_T_MIN, MIT_T_MAX, 12);
    if (msg.data_length_code >= 8) { ak[k].mtemp = msg.data[6]; ak[k].merr = msg.data[7]; }
    ak[k].mit_have = true;
    return;
  }
}

// vel+accel limited ramp of mit_pdes -> mit_pgoal, then stream MIT setpoint.
// Onboard motor does kp*(p_des-p)+kd*(0-v)+tff.  (from hw_selftest mit_step)
static void mit_step(AK40Motor &m, float dt) {
  if (!m.enabled || !m.mit_have) return;
  float perr = m.mit_pgoal - m.mit_pdes;
  float dir  = (perr >= 0.0f) ? 1.0f : -1.0f;
  float dist = fabsf(perr);
  float vcap = sqrtf(2.0f * MIT_AMAX * dist);
  if (vcap > MIT_VMAX) vcap = MIT_VMAX;
  m.mit_pvel += clampf(dir * vcap - m.mit_pvel, -MIT_AMAX * dt, MIT_AMAX * dt);
  m.mit_pdes += m.mit_pvel * dt;
  if (dist < 0.5f * DEG2RAD && fabsf(m.mit_pvel) < 1.0f * DEG2RAD) {
    m.mit_pdes = m.mit_pgoal; m.mit_pvel = 0.0f;
  }
  // tracking-error e-stop: debounced (ignore single-frame CAN/theta glitches) and
  // suspended during a thermal limp (which is intentionally sagging).
  if (!m.thermal && fabsf(m.mit_pdes - m.theta) > POS_ERR_LIMIT_DEG * DEG2RAD) {
    if (++m.estop_cnt >= ESTOP_DEBOUNCE) { m.enabled = false; mit_special(m.can_id, 0xFD); return; }
  } else {
    m.estop_cnt = 0;
  }
  float kd = MIT_KD_HOLD;
  if (m.mit_pvel != 0.0f && MIT_KD_JOG > kd) kd = MIT_KD_JOG;       // boost while ramping
  mit_pack(m.can_id, m.mit_pdes, 0.0f, g_kp_ramp * m.kp_scale, kd, MIT_TFF);
}

static bool twai_setup() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

// enter MIT motor mode on both AK40 (idempotent); they then reply with theta.
static void ak_enter_mode() {
  if (DRY_RUN || !can_ok) return;
  for (int i = 0; i < AK40_NUM; i++) { mit_special(ak[i].can_id, 0xFC); ak[i].mit_have = false; }
  can_entered = true;
}
// limp both motors (exit motor mode) -- safe state.
static void ak_limp() {
  for (int i = 0; i < AK40_NUM; i++) ak[i].enabled = false;
  if (DRY_RUN || !can_ok) return;
  for (int i = 0; i < AK40_NUM; i++) mit_special(ak[i].can_id, 0xFD);
  can_entered = false;
}

// ─── NVS home offset ─────────────────────────────────────────────────────────
static void nvs_load_offsets() {
  prefs.begin("arm", true);
  for (int i = 0; i < AK40_NUM; i++) {
    char key[8]; snprintf(key, sizeof(key), "off%d", i);
    ak[i].offset = prefs.getFloat(key, 0.0f);
  }
  prefs.end();
}
static void nvs_save_offsets() {
  prefs.begin("arm", false);
  for (int i = 0; i < AK40_NUM; i++) {
    char key[8]; snprintf(key, sizeof(key), "off%d", i);
    prefs.putFloat(key, ak[i].offset);
  }
  prefs.end();
}

// ─── callbacks ───────────────────────────────────────────────────────────────
// /joint_commands : map shoulder/elbow rad -> motor goal (clamped). base_yaw,
// wrist_roll ignored (no motor); wrist_pitch ignored (autonomous leveling).
static void cmd_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *m = (const sensor_msgs__msg__JointState *)msgin;
  if (m->position.size > SH_JIDX) {
    float q = clampf((float)m->position.data[SH_JIDX], JLIM_LO[SH_JIDX], JLIM_HI[SH_JIDX]);
    g_cmd[SH_JIDX] = q; ak[0].mit_pgoal = theta_from_q(0, q);
  }
  if (m->position.size > EL_JIDX) {
    float q = clampf((float)m->position.data[EL_JIDX], JLIM_LO[EL_JIDX], JLIM_HI[EL_JIDX]);
    g_cmd[EL_JIDX] = q; ak[1].mit_pgoal = theta_from_q(1, q);
  }
  last_cmd_ms = millis();
}

// /home : capture current raw theta as the q=0 reference, persist to NVS, and
// re-anchor the hold goal so the arm does not jump.  Pull arm vertical first.
static void home_callback(const void *msgin) {
  (void)msgin;
  for (int i = 0; i < AK40_NUM; i++) {
    ak[i].offset   = ak[i].theta;          // raw at vertical -> q=0 here
    ak[i].mit_pgoal = ak[i].mit_pdes = ak[i].theta;
    ak[i].mit_pvel = 0.0f;
  }
  nvs_save_offsets();
}

// /arm_enable : true -> (re)enter motor mode and hold at current pose; false -> limp.
static void enable_callback(const void *msgin) {
  const std_msgs__msg__Bool *b = (const std_msgs__msg__Bool *)msgin;
  if (b->data) {
    ak_enter_mode();
    g_kp_ramp = 0.0f;                                   // kp soft-start: ramp up from 0
    for (int i = 0; i < AK40_NUM; i++) {
      ak[i].mit_pgoal = ak[i].mit_pdes = ak[i].theta;   // hold where it is (no jump)
      ak[i].mit_pvel = 0.0f; ak[i].enabled = true;
      ak[i].kp_scale = 1.0f; ak[i].thermal = false; ak[i].estop_cnt = 0;  // clear thermal/e-stop
    }
  } else {
    ak_limp();
  }
}

static void wrist_level_start();
static void wrist_level_stop();

// /level_enable : true -> start wrist cup-leveling; false -> motor off.  Independent
// of /arm_enable so the wrist can be sign-verified at low Kp with the AK40s still limp.
static void level_callback(const void *msgin) {
  const std_msgs__msg__Bool *b = (const std_msgs__msg__Bool *)msgin;
  if (b->data) wrist_level_start();
  else         wrist_level_stop();
}

static void set_wrist_eso_bw(float wo) {       // Gao triple pole at -wo
  wr_eso_wo = wo;
  wr_eso_l1 = 3.0f * wo;
  wr_eso_l2 = 3.0f * wo * wo;
  wr_eso_l3 = wo * wo * wo;
}

static void set_wrist_J(float J) {             // update inertia -> recompute b0 = Kt/J
  if (J > 1e-6f) { wrist_J = J; wr_eso_b0 = WR_DOB_KT / J; }
}

// /wrist_cfg : live tuning [kp, kd, dob_scale, dob_bw_hz, dob_on, J_nom, kp2, kd2]. Each
// element optional (applied only if present).  J_nom switches the ESO plant model
// for cup-on vs cup-off (b0 = Kt/J).  kp2/kd2 = motor2(roll) gains (heavier axis).
// One subscription, wider array (buffer capacity 8) -> no extra sub.  Replaces /set_wrist_kp.
static void cfg_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  size_t n = m->data.size;
  if (n > 0) wrist_kp        = clampf(m->data.data[0], 0.0f, 40.0f);
  if (n > 1) wrist_kd        = clampf(m->data.data[1], 0.0f, 5.0f);
  if (n > 2) wrist_dob_scale = clampf(m->data.data[2], -3.0f, 3.0f);
  if (n > 3) set_wrist_eso_bw(2.0f * PI * clampf(m->data.data[3], 0.5f, 9.0f));  // keep < ~9Hz resonance
  if (n > 4) {
    bool on = (m->data.data[4] > 0.5f);
    if (on && !wrist_dob_on) {                   // rising edge: reseed ESO (was idle while off)
      wr_eso_x1 = wrist_enc.getAngle(); wr_eso_x2 = 0; wr_eso_x3 = 0;
      wr_dhat = 0; wr_iq_filt = 0; wr_last_iq_us = micros();
    }
    wrist_dob_on = on;
  }
  if (n > 5) set_wrist_J(clampf(m->data.data[5], 3e-5f, 2e-3f));
  if (n > 6) wrist_kp2 = clampf(m->data.data[6], 0.0f, 40.0f);   // roll gains
  if (n > 7) wrist_kd2 = clampf(m->data.data[7], 0.0f, 5.0f);
}

// ─── wrist_pitch leveling (SimpleFOC + ICM20948) ─────────────────────────────
static inline float wrist_accel_phi(float ax, float ay) { return atan2f(-ax, -ay); }
// 2nd axis tilt about IMU x (gimbal_level_hold mapping, -y up).  ⚠️ VERIFY on HW.
static inline float wrist_accel_phi2(float az, float ay) { return atan2f(-az, -ay); }

static bool wr_encoder_alive(uint8_t cs) {
  for (int t = 0; t < 4; t++) {
    SPI.beginTransaction(SPISettings(WR_ENC_SPI_HZ, MSBFIRST, SPI_MODE1));
    digitalWrite(cs, LOW);
    uint16_t raw = SPI.transfer16(0xFFFF);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    if (raw != 0xFFFF && raw != 0x0000) return true;
    delayMicroseconds(100);
  }
  return false;
}

// initFOC with optional align-skip.  If cal_zero is set (!= NO_CAL): inject the stored
// zero/dir so initFOC does NOT rotate (no boot spin).  Else: align (rotates) and print
// the measured values to copy into WR*_CAL_*.
static bool wr_init_foc(BLDCMotor &m, uint8_t cs, float cal_zero, int cal_dir, const char *tag) {
  if (!wr_encoder_alive(cs)) return false;
  if (cal_zero < NO_CAL) {                          // stored cal -> skip align, no rotation
    m.sensor_direction   = (cal_dir >= 0) ? Direction::CW : Direction::CCW;
    m.zero_electric_angle = cal_zero;
    m.initFOC();
    Serial.printf("%s stored cal: zero=%.4f dir=%d (align skipped)\n", tag, cal_zero, cal_dir);
    return true;
  }
  for (int i = 0; i < 5; i++) {                      // uncalibrated -> align (rotates)
    m.initFOC();
    if (m.sensor_direction != Direction::UNKNOWN && m.zero_electric_angle > -1000.0f) {
      Serial.printf("%s cal: zero=%.4f dir=%d  <-- copy into WR*_CAL_*\n",
                    tag, m.zero_electric_angle, (int)m.sensor_direction);
      return true;
    }
    if (!wr_encoder_alive(cs)) break;
    delay(300);
  }
  return false;
}

static bool wr_setup_imu() {
  wrist_imu.begin(WR_IMU_CS, SPI, WR_IMU_SPI_HZ);
  if (wrist_imu.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fss; fss.a = gpm4; fss.g = dps500;
  wrist_imu.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_dlpcfg_t dlp; dlp.a = acc_d50bw4_n68bw8; dlp.g = gyr_d51bw2_n73bw3;
  wrist_imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);
  wrist_imu.enableDLPF(ICM_20948_Internal_Acc, true);
  wrist_imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  ICM_20948_smplrt_t srd; srd.a = 2; srd.g = 2;   // ODR 1125/(1+2)=375Hz > 300Hz poll = fresh
  wrist_imu.setSampleRate(ICM_20948_Internal_Acc, srd);
  wrist_imu.setSampleRate(ICM_20948_Internal_Gyr, srd);
  return (wrist_imu.status == ICM_20948_Stat_Ok);
}

static void wr_calib_gyro() {        // 2s still at boot -> gyro z/x bias + seed est_phi
  double sz = 0, sx = 0; int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    wrist_imu.getAGMT();             // no dataReady gate (see attitude_tick)
    sz += wrist_imu.gyrZ(); sx += wrist_imu.gyrX(); n++;
    delay(3);
  }
  if (n > 0) { wrist_gbias_z = (float)(sz / n) * (PI / 180.0f);
               wrist_gbias_x = (float)(sx / n) * (PI / 180.0f); }
  wrist_imu.getAGMT();
  wrist_est_phi  = wrist_accel_phi (wrist_imu.accX(), wrist_imu.accY());
  wrist_est_phi2 = wrist_accel_phi2(wrist_imu.accZ(), wrist_imu.accY());
}

static void setup_wrist() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  // park every CS HIGH on the shared bus (incl. unused ch2) -- a floating CS corrupts SPI
  pinMode(WR_ENC_CS,  OUTPUT); digitalWrite(WR_ENC_CS,  HIGH);
  pinMode(WR_ENC2_CS, OUTPUT); digitalWrite(WR_ENC2_CS, HIGH);
  pinMode(WR_IMU_CS,  OUTPUT); digitalWrite(WR_IMU_CS,  HIGH);
  pinMode(22, INPUT);   // damaged GPIO22 (internal short to MOSI/23): never drive it
  wrist_enc.init(&SPI);                       // encoder first (cold-bus warm), then IMU
  // IMU init NOW, on the warm but otherwise-clean bus, BEFORE the motor initFOC (esp.
  // the dead-phase motor2 align) which seemed to leave the IMU init failing.
  for (int i = 0; i < 5 && !wrist_imu_ok; i++) {
    wrist_imu_ok = wr_setup_imu();
    if (!wrist_imu_ok) delay(80);
  }
  if (wrist_imu_ok) wr_calib_gyro();
  wrist_motor.linkSensor(&wrist_enc);
  wrist_drv.voltage_power_supply = WR_SUPPLY_V;
  wrist_drv.voltage_limit = WR_DRIVER_VLIMIT;
  wrist_drv.init();
  wrist_motor.linkDriver(&wrist_drv);
  // current sense for ESO Iq estimate (voltage torque mode; sense read-only)
  wrist_cs.linkDriver(&wrist_drv); wrist_cs.init();
  wrist_cs.gain_a *= -1; wrist_cs.gain_b *= -1; wrist_cs.skip_align = true;  // gimbal_level_hold convention
  wrist_motor.linkCurrentSense(&wrist_cs);
  wrist_motor.voltage_limit   = WR_VLIMIT;
  wrist_motor.current_limit   = WR_ILIMIT;
  wrist_motor.velocity_limit  = WR_VEL_LIMIT;
  wrist_motor.voltage_sensor_align = 5.0f;
  wrist_motor.controller       = MotionControlType::torque;
  wrist_motor.torque_controller = TorqueControlType::voltage;   // PD outputs voltage; Iq read for ESO only
  wrist_motor.init();
  set_wrist_eso_bw(wr_eso_wo);                  // seed l1/l2/l3 from default 5Hz
  wrist_ok = wr_init_foc(wrist_motor, WR_ENC_CS, WR_CAL_ZERO, WR_CAL_DIR, "WRIST");
  wrist_motor.disable();

#if WR2_ENABLE
  // ---- motor2 (wrist_roll), mirror of motor1 ----
  wrist_enc2.init(&SPI);
  wrist_motor2.linkSensor(&wrist_enc2);
  wrist_drv2.voltage_power_supply = WR_SUPPLY_V;
  wrist_drv2.voltage_limit = WR_DRIVER_VLIMIT;
  wrist_drv2.init();
  wrist_motor2.linkDriver(&wrist_drv2);
  wrist_cs2.linkDriver(&wrist_drv2); wrist_cs2.init();
  wrist_cs2.gain_a *= -1; wrist_cs2.gain_b *= -1; wrist_cs2.skip_align = true;
  wrist_motor2.linkCurrentSense(&wrist_cs2);
  wrist_motor2.voltage_limit   = WR_VLIMIT;
  wrist_motor2.current_limit   = WR_ILIMIT;
  wrist_motor2.velocity_limit  = WR_VEL_LIMIT;
  wrist_motor2.voltage_sensor_align = 8.0f;   // raised 5->8 (replaced roll motor, weak align under gravity load). 8V/5.5ohm~=1.45A < ILIMIT 2A
  wrist_motor2.controller       = MotionControlType::torque;
  wrist_motor2.torque_controller = TorqueControlType::voltage;
  wrist_motor2.init();
  wrist_ok2 = wr_init_foc(wrist_motor2, WR2_ENC_CS, WR2_CAL_ZERO, WR2_CAL_DIR, "ROLL");
  wrist_motor2.disable();
#else
  // motor2 unused (dead roll): park its PWM/EN pins LOW so they don't float and twitch
  // the MKS driver (whine).  SimpleFOC isn't owning them when WR2_ENABLE=0.
  pinMode(WR2_PWM_A, OUTPUT); digitalWrite(WR2_PWM_A, LOW);
  pinMode(WR2_PWM_B, OUTPUT); digitalWrite(WR2_PWM_B, LOW);
  pinMode(WR2_PWM_C, OUTPUT); digitalWrite(WR2_PWM_C, LOW);
  pinMode(WR2_EN,    OUTPUT); digitalWrite(WR2_EN,    LOW);
#endif // WR2_ENABLE
}

static void wrist_level_start() {
  if (!wrist_ok || !wrist_imu_ok) return;
  // NOTE: do NOT re-seed est_phi from a single accel read here -- that one raw
  // sample glitches (saw -130deg spikes -> instant ATT_LIMIT trip).  The running
  // complementary estimate (attitude_tick) is already stable; just use it.
  wrist_enc0 = wrist_enc.getAngle();          // q=0 reference for /joint_states
  wrist_vpd = 0.0f;
  wr_eso_x1 = wrist_enc.getAngle();           // fresh ESO: seed position, rest 0
  wr_eso_x2 = 0; wr_eso_x3 = 0; wr_dhat = 0;
  wr_iq_filt = 0.0f; wr_last_iq_us = micros();
  wrist_motor.enable();
  if (wrist_ok2) {                            // 2nd axis (wrist_roll)
    wrist_enc0_2 = wrist_enc2.getAngle();
    wrist_vpd2 = 0.0f;
    wr2_eso_x1 = wrist_enc2.getAngle(); wr2_eso_x2 = 0; wr2_eso_x3 = 0; wr2_dhat = 0;
    wr2_iq_filt = 0.0f; wr2_last_iq_us = micros();
    wrist_motor2.enable();
  }
  wrist_hold = true;
}

static void wrist_level_stop() {
  wrist_hold = false;
  wrist_vpd  = 0.0f;
  wrist_motor.target = 0.0f;
  wrist_motor.disable();
  if (wrist_ok2) {                 // motor2 only if it was actually set up (WR2_ENABLE)
    wrist_vpd2 = 0.0f;
    wrist_motor2.target = 0.0f;
    wrist_motor2.disable();
  }
}

// FOC current loop + ESO: must run EVERY loop iteration (fast/kHz) for smooth torque
// and so wo*dt stays small.  ESO observes disturbance from encoder theta + Iq; the
// compensation voltage is added to the PD output (the project's core DOB feature).
static void wrist_foc_tick() {
  if (!wrist_ok) return;
  wrist_enc.update();
  if (!wrist_motor.enabled) return;
  wrist_motor.loopFOC();                        // electrical_angle valid after this

  float v = wrist_vpd;
  // ESO/current-sense ONLY when DOB on: getFOCCurrents() does 2 ESP32 analogReads
  // (~100us each) which halves the loop rate.  PD-only path skips it -> ~kHz loop.
  if (wrist_dob_on) {
    unsigned long nu = micros();
    float dtq = (nu - wr_last_iq_us) * 1e-6f; wr_last_iq_us = nu;
    if (dtq > 0.0f && dtq < 0.05f) {
      float iqr = wrist_cs.getFOCCurrents(wrist_motor.electrical_angle).q;
      wr_iq_filt += (dtq / (WR_IQ_TAU + dtq)) * (iqr - wr_iq_filt);
      float th  = wrist_enc.getAngle();
      float err = th - wr_eso_x1;                 // ESO predict/correct (Gao triple pole)
      wr_eso_x1 += dtq * (wr_eso_x2 + wr_eso_l1 * err);
      wr_eso_x2 += dtq * (wr_eso_b0 * wr_iq_filt + wr_eso_x3 + wr_eso_l2 * err);
      wr_eso_x3 += dtq * (wr_eso_l3 * err);
      wr_dhat = wrist_J * wr_eso_x3;              // disturbance torque estimate [N·m]
      float comp = -wrist_dob_scale * (WR_DOB_R / WR_DOB_KT) * wr_dhat;  // torque->voltage
      v += clampf(comp, -WR_DOB_VCLAMP, WR_DOB_VCLAMP);
    }
  } else {
    wr_last_iq_us = micros();                     // keep dtq sane for when DOB re-enables
    wr_dhat = 0.0f;
  }
  wrist_motor.move(clampf(v, -WR_VLIMIT, WR_VLIMIT));

  // ---- motor2 (wrist_roll), same structure ----
  if (!wrist_ok2) return;
  wrist_enc2.update();
  if (!wrist_motor2.enabled) return;
  wrist_motor2.loopFOC();
  float v2 = wrist_vpd2;
  if (wrist_dob_on) {
    unsigned long nu = micros();
    float dtq = (nu - wr2_last_iq_us) * 1e-6f; wr2_last_iq_us = nu;
    if (dtq > 0.0f && dtq < 0.05f) {
      float iqr = wrist_cs2.getFOCCurrents(wrist_motor2.electrical_angle).q;
      wr2_iq_filt += (dtq / (WR_IQ_TAU + dtq)) * (iqr - wr2_iq_filt);
      float th  = wrist_enc2.getAngle();
      float err = th - wr2_eso_x1;
      wr2_eso_x1 += dtq * (wr2_eso_x2 + wr_eso_l1 * err);   // shared l1/l2/l3/b0/J
      wr2_eso_x2 += dtq * (wr_eso_b0 * wr2_iq_filt + wr2_eso_x3 + wr_eso_l2 * err);
      wr2_eso_x3 += dtq * (wr_eso_l3 * err);
      wr2_dhat = wrist_J * wr2_eso_x3;
      float comp = -wrist_dob_scale * (WR_DOB_R / WR_DOB_KT) * wr2_dhat;
      v2 += clampf(comp, -WR_DOB_VCLAMP, WR_DOB_VCLAMP);
    }
  } else {
    wr2_last_iq_us = micros();
    wr2_dhat = 0.0f;
  }
  wrist_motor2.move(clampf(v2, -WR_VLIMIT, WR_VLIMIT));
}

// Attitude loop @300Hz: estimate cup tilt, PD toward level (target=0) -> wrist_vpd.
static void wrist_attitude_tick() {
  if (!wrist_imu_ok) return;
  unsigned long now = micros();
  if (now - wrist_last_imu_us < WR_IMU_PERIOD_US) return;
  float dt = (now - wrist_last_imu_us) * 1e-6f;
  wrist_last_imu_us = now;

  // NOTE: no dataReady() gate -- it was never returning true on this MCU and froze
  // est_phi.  getAGMT() reads the current registers regardless (ODR 375Hz > 300Hz tick).
  wrist_imu.getAGMT();
  float gz = wrist_imu.gyrZ() * (PI / 180.0f) - wrist_gbias_z;
  wrist_est_phi += gz * dt;
  float ax = wrist_imu.accX(), ay = wrist_imu.accY(), az = wrist_imu.accZ();
  float anorm = sqrtf(ax*ax + ay*ay + az*az);
  if (anorm > 700.0f && anorm < 1300.0f)      // reject high-accel (only trust gravity)
    wrist_est_phi = WR_COMP_ALPHA * wrist_est_phi
                  + (1.0f - WR_COMP_ALPHA) * wrist_accel_phi(ax, ay);

  // 2nd axis: tilt about IMU x -> motor2 (wrist_roll).  reuse ax/ay/az from above.
  float gx = WR_GYRO2_SIGN * (wrist_imu.gyrX() * (PI / 180.0f) - wrist_gbias_x);
  wrist_est_phi2 += gx * dt;
  if (anorm > 700.0f && anorm < 1300.0f)
    wrist_est_phi2 = WR_COMP_ALPHA * wrist_est_phi2
                   + (1.0f - WR_COMP_ALPHA) * wrist_accel_phi2(az, ay);

  if (wrist_hold) {
    if (fabsf(wrist_est_phi) > WR_ATT_LIMIT) { wrist_level_stop(); return; }  // diverged -> e-stop
    wrist_vpd = WR_SIGN * (wrist_kp * (0.0f - wrist_est_phi) - wrist_kd * gz); // target = level
    if (wrist_ok2) {
      if (fabsf(wrist_est_phi2) > WR_ATT_LIMIT) { wrist_level_stop(); return; }
      wrist_vpd2 = WR_SIGN2 * (wrist_kp2 * (0.0f - wrist_est_phi2) - wrist_kd2 * gx);
    }
  }
}

// ─── AK40 200Hz control tick ─────────────────────────────────────────────────
static void ak_control_tick() {
  uint32_t now = micros();
  if ((uint32_t)(now - last_ak_us) < AK40_CTRL_US) return;
  float dt = (now - last_ak_us) * 1e-6f;
  last_ak_us = now;

  // kp soft-start: slew the streamed stiffness toward target so enable engages
  // gently (no hard catch).  also smooths live /set_kp changes.
  float kstep = KP_SLEW * dt;
  if (g_kp_ramp < g_mit_kp)      g_kp_ramp = fminf(g_kp_ramp + kstep, g_mit_kp);
  else if (g_kp_ramp > g_mit_kp) g_kp_ramp = fmaxf(g_kp_ramp - kstep, g_mit_kp);

#if DRY_RUN
  // simulate: theta follows goal (offset/dir = identity), so q echoes command
  for (int i = 0; i < AK40_NUM; i++) {
    ak[i].mit_have = true;
    ak[i].theta += 0.25f * (ak[i].mit_pgoal - ak[i].theta);
  }
#else
  if (!can_ok) return;
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) parseMitReply(rx);   // drain replies -> theta

  // boot home-recovery: once, on first valid reading, snap stored offset to the
  // sector nearest the current theta (arm must be at home +-half sector at connect).
  if (!offset_snapped && ak[0].mit_have && ak[1].mit_have) {
    for (int i = 0; i < AK40_NUM; i++)
      ak[i].offset += ARM_SECTOR_RAD * roundf((ak[i].theta - ak[i].offset) / ARM_SECTOR_RAD);
    offset_snapped = true;
  }

  // SW thermal guard: above TEMP_LIMP, gently ramp that motor's kp to 0 (controlled
  // sag) then limp -- pre-empts the motor's own abrupt hard trip (floor slam).
  for (int i = 0; i < AK40_NUM; i++) {
    if (ak[i].enabled && ak[i].mtemp >= TEMP_LIMP) ak[i].thermal = true;
    if (ak[i].thermal && ak[i].enabled) {
      ak[i].kp_scale -= THERMAL_KP_RATE * dt;
      if (ak[i].kp_scale <= 0.0f) { ak[i].kp_scale = 0.0f; ak[i].enabled = false; mit_special(ak[i].can_id, 0xFD); }
    }
  }

  for (int i = 0; i < AK40_NUM; i++) {
    if (ak[i].enabled) mit_step(ak[i], dt);
    else if (can_entered) mit_pack(ak[i].can_id, 0, 0, 0, 0, 0);  // zero-gain: backdrivable + theta live
  }
#endif
}

// fill g_state from motor theta + placeholders, publish /joint_states.
static void publish_state() {
  if (millis() - last_state_ms < STATE_PERIOD_MS) return;
  last_state_ms = millis();

  bool fresh = (millis() - last_cmd_ms) < CMD_WATCHDOG_MS; (void)fresh;
  g_state[0] = 0.0f;                                   // base_yaw : no motor
  g_state[SH_JIDX] = q_from_theta(0, ak[0].theta);     // shoulder
  g_state[EL_JIDX] = q_from_theta(1, ak[1].theta);     // elbow
  // wrist_pitch : autonomous IMU level-hold (SimpleFOC + ICM20948 on this MCU).
  // Report actual joint angle from the wrist encoder (relative to level-enable
  // reference).  Falls back to 0 if FOC init failed.
  g_state[WP_JIDX] = wrist_ok
      ? clampf(wrist_enc.getAngle() - wrist_enc0, JLIM_LO[WP_JIDX], JLIM_HI[WP_JIDX])
      : 0.0f;
  g_state[4] = wrist_ok2                               // wrist_roll : motor2 encoder
      ? clampf(wrist_enc2.getAngle() - wrist_enc0_2, JLIM_LO[4], JLIM_HI[4])
      : 0.0f;

  int64_t ms = rmw_uros_epoch_millis();
  state_msg.header.stamp.sec     = (int32_t)(ms / 1000);
  state_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);
  for (int i = 0; i < NUM_JOINTS; i++) state_msg.position.data[i] = (double)g_state[i];
  state_msg.effort.data[SH_JIDX] = (double)ak[0].mit_tau;   // expose motor torque
  state_msg.effort.data[EL_JIDX] = (double)ak[1].mit_tau;
  // wrist leveling debug (spare effort fields):
  //   effort[0] = wrist status bits: 1=foc_ok, 2=imu_ok, 4=holding, 8=dob_on, 16=foc_ok2(roll)
  //   effort[3] = pitch est_phi [rad]   effort[4] = pitch vpd [V]
  //   DOB-verify telemetry: [5]=pitch d_hat[Nm] [6]=pitch iq[A] [7]=roll est_phi2 [8]=roll vpd2
  //                         [9]=roll d_hat [10]=roll iq
  state_msg.effort.data[0] = (double)((wrist_ok?1:0) | (wrist_imu_ok?2:0) | (wrist_hold?4:0)
                                      | (wrist_dob_on?8:0) | (wrist_ok2?16:0));
  state_msg.effort.data[WP_JIDX] = (double)wrist_est_phi;
  state_msg.effort.data[4]  = (double)wrist_vpd;
  state_msg.effort.data[5]  = (double)wr_dhat;
  state_msg.effort.data[6]  = (double)wr_iq_filt;
  state_msg.effort.data[7]  = (double)wrist_est_phi2;
  state_msg.effort.data[8]  = (double)wrist_vpd2;
  state_msg.effort.data[9]  = (double)wr2_dhat;
  state_msg.effort.data[10] = (double)wr2_iq_filt;
  // motor temp[C] + error + thermal flags via the (otherwise unused) velocity field:
  //   velocity = [flags, temp_sh, temp_el, err_sh, err_el]
  //   flags bits: 1=sh thermal-limp, 2=el thermal-limp, 4=sh warn, 8=el warn
  state_msg.velocity.data[0] = (double)((ak[0].thermal?1:0) | (ak[1].thermal?2:0)
                                      | (ak[0].mtemp>=TEMP_WARN?4:0) | (ak[1].mtemp>=TEMP_WARN?8:0));
  state_msg.velocity.data[1] = (double)ak[0].mtemp;
  state_msg.velocity.data[2] = (double)ak[1].mtemp;
  state_msg.velocity.data[3] = (double)ak[0].merr;
  state_msg.velocity.data[4] = (double)ak[1].merr;
  RCSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));
}

// ─── JointState message memory ───────────────────────────────────────────────
static void alloc_jointstate(sensor_msgs__msg__JointState *m, int n, int strcap) {
  m->position.data = (double *)calloc(n, sizeof(double));
  m->position.size = n; m->position.capacity = n;
  m->velocity.data = (double *)calloc(n, sizeof(double));
  m->velocity.size = n; m->velocity.capacity = n;
  m->effort.data = (double *)calloc(n, sizeof(double));
  m->effort.size = n; m->effort.capacity = n;
  m->name.data = (rosidl_runtime_c__String *)calloc(n, sizeof(rosidl_runtime_c__String));
  m->name.size = n; m->name.capacity = n;
  for (int i = 0; i < n; i++) {
    m->name.data[i].data = (char *)calloc(strcap, 1);
    m->name.data[i].capacity = strcap; m->name.data[i].size = 0;
  }
  m->header.frame_id.data = (char *)calloc(strcap, 1);
  m->header.frame_id.capacity = strcap; m->header.frame_id.size = 0;
}

// ─── entity (re)creation ─────────────────────────────────────────────────────
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "gimbalarm_mcu", "", &support));

  // reliable: matches robot_state_publisher + standard /joint_states consumers
  // (RViz RobotModel goes red on a best_effort pub -> reliable sub mismatch).
  // best_effort was a wrong guess for the 1Hz bug; real fix = spin_some(...,0),
  // which is QoS-independent, so reliable here is safe.
  RCCHECK(rclc_publisher_init_default(
      &state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));
  RCCHECK(rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_commands"));
  RCCHECK(rclc_subscription_init_default(
      &home_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/home"));
  RCCHECK(rclc_subscription_init_default(
      &enable_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/arm_enable"));
  RCCHECK(rclc_subscription_init_default(
      &level_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/level_enable"));
  RCCHECK(rclc_subscription_init_default(
      &cfg_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wrist_cfg"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub,    &cmd_msg,    &cmd_callback,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &home_sub,   &home_msg,   &home_callback,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &enable_sub, &enable_msg, &enable_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &level_sub,  &level_msg,  &level_callback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cfg_sub,    &cfg_msg,    &cfg_callback,    ON_NEW_DATA));

  for (int i = 0; i < NUM_JOINTS; i++)
    rosidl_runtime_c__String__assign(&state_msg.name.data[i], JOINT_NAMES[i]);
  rosidl_runtime_c__String__assign(&state_msg.header.frame_id, "");

  rmw_uros_sync_session(1000);
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  (void)rcl_subscription_fini(&cmd_sub, &node);
  (void)rcl_subscription_fini(&home_sub, &node);
  (void)rcl_subscription_fini(&enable_sub, &node);
  (void)rcl_subscription_fini(&level_sub, &node);
  (void)rcl_subscription_fini(&cfg_sub, &node);
  (void)rcl_publisher_fini(&state_pub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

// ─── setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);                // begin BEFORE setup_wrist so cal prints are readable
                                       // (set_microros_transports re-begins same baud later, harmless)
  park_gimbal();                       // brownout guard first

  for (int i = 0; i < AK40_NUM; i++) {
    ak[i] = AK40Motor();
    ak[i].can_id = AK40_CAN_ID[i];
    ak[i].dir    = AK40_DIR[i];
  }
  nvs_load_offsets();

  // IMU/SPI init BEFORE CAN -- hw_selftest does this order and the IMU inits reliably;
  // bringing up TWAI first (its ISR/bus activity) disturbed the delicate IMU WHO_AM_I/
  // config SPI writes -> IMU FAIL.  Sensors first, then CAN.
#if WRIST_ENABLE
  setup_wrist();                       // GM4108 FOC + ICM20948 (SPI) + gyro calib (2s still)
  // cal-dump window: only while uncalibrated (NO_CAL).  Prints zero/dir for ~10s so it's
  // catchable on a 921600 terminal before the micro-ROS binary stream starts in loop().
  // Fill WR*_CAL_* with these -> window skips (no delay) AND initFOC skips align (no boot spin).
  if (WR_CAL_ZERO >= NO_CAL || WR2_CAL_ZERO >= NO_CAL) {
    for (int i = 0; i < 20; i++) {
      Serial.printf(">>> CAL  WRIST zero=%.4f dir=%d   ROLL zero=%.4f dir=%d\n",
        wrist_motor.zero_electric_angle,  (int)wrist_motor.sensor_direction,
        wrist_motor2.zero_electric_angle, (int)wrist_motor2.sensor_direction);
      delay(500);
    }
  }
#endif

#if !DRY_RUN
  can_ok = twai_setup();
#endif

  set_microros_transports();
  alloc_jointstate(&state_msg, NUM_JOINTS, 24);
  alloc_jointstate(&cmd_msg,   NUM_JOINTS, 24);
  // widen state_msg.effort for DOB-verification telemetry (indices 5..10).  position/name
  // stay NUM_JOINTS so RViz/robot_state_publisher are unaffected.
  free(state_msg.effort.data);
  state_msg.effort.data = (double *)calloc(16, sizeof(double));
  state_msg.effort.size = 11; state_msg.effort.capacity = 16;

  // /wrist_cfg incoming buffer (Float32MultiArray, dynamic) -- pre-allocate data seq;
  // publisher sends an empty layout so dim seq stays at 0 capacity.
  cfg_msg.data.data = (float *)calloc(8, sizeof(float));
  cfg_msg.data.size = 0; cfg_msg.data.capacity = 8;
  cfg_msg.layout.dim.data = NULL; cfg_msg.layout.dim.size = 0; cfg_msg.layout.dim.capacity = 0;
  cfg_msg.layout.data_offset = 0;

  agent_state = WAITING_AGENT;
  last_ak_us  = micros();
}

// ─── loop: non-blocking connection state machine ─────────────────────────────
void loop() {
  switch (agent_state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, {
      agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
    });
    break;

  case AGENT_AVAILABLE:
    if (create_entities()) {
      ak_enter_mode();                 // motors in MIT mode, still limp (enabled=false)
      agent_state = AGENT_CONNECTED;
    } else {
      destroy_entities();
      agent_state = WAITING_AGENT;
    }
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(2000, {   // agent liveliness check. 1 attempt/100ms (was 3 = up to
      // 300ms loop stall/sec -> control-loop jitter).  every 2s, single try, low timeout.
      agent_state = (rmw_uros_ping_agent(60, 1) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    });
    if (agent_state == AGENT_CONNECTED) {
#if WRIST_ENABLE
      // auto-recover a flaky IMU (cold-SPI init fail) without a reset; no 2s calib here
      if (!wrist_imu_ok) {
        EXECUTE_EVERY_N_MS(1500, {
          wrist_imu_ok = wr_setup_imu();
          if (wrist_imu_ok) { wrist_imu.getAGMT();
            wrist_est_phi = wrist_accel_phi(wrist_imu.accX(), wrist_imu.accY()); }
        });
      }
      wrist_foc_tick();                // every iteration (fast FOC current loop)
      wrist_attitude_tick();           // 300Hz internal gate (IMU PD -> wrist_vpd)
#endif
      ak_control_tick();               // 200Hz internal gate
      publish_state();                 // 25Hz internal gate
      // 0 timeout = non-blocking poll.  A positive timeout made the XRCE serial
      // transport read block ~1s when idle (no incoming cmd), throttling the whole
      // loop to 1Hz (the passed 2ms was NOT honored).  0 = take a different rcl_wait
      // path: process already-arrived data, return immediately.
      // Gated to ~400Hz: rcl_wait setup every iteration was overhead that starved the
      // wrist FOC (which must free-run at kHz).  400Hz is ample for incoming setpoints.
      static uint32_t last_spin_us = 0;
      uint32_t nowu = micros();
      if ((uint32_t)(nowu - last_spin_us) >= 2500) {
        last_spin_us = nowu;
        rclc_executor_spin_some(&executor, 0);
      }
    }
    break;

  case AGENT_DISCONNECTED:
    ak_limp();                         // motors safe while orphaned
    wrist_level_stop();                // cup leveling off when orphaned
    park_gimbal();
    destroy_entities();
    agent_state = WAITING_AGENT;
    break;
  }
}
