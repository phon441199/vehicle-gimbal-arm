/*
 * AK40-10 x2 (CMESC Servo) joint IMPEDANCE + DOB over CAN — Teensy 4.1
 *
 * Transceiver WCMCU230 @5V, RX via 1k+2k divider (5V->3.3V), TX direct.
 * Teensy CAN1:  CTX1 = pin 22 -> WCMCU230 TXD
 *               CRX1 = pin 23 <- (divider) <- WCMCU230 RXD
 * WCMCU230 VCC -> 5V (VIN), GND -> GND (common with motor PSU).
 * Daisy-chain bus, 120Ω at both physical ends only (verify CANH-CANL = 60Ω).
 *
 * Protocol: AK Series Manual V1.0.18 (TX: id|(pkt<<8) extended; status 0x29|id).
 *
 * Serial @115200:
 *   m1 / m2 / ma   select target motor (ma = both)
 *   e   enable pure impedance (DOB off)      d   enable impedance+DOB
 *   o   toggle DOB        s   stop/disable
 *   t<deg>  target    k<val> K    b<val> D
 *   j<val>  DOB J_nom g<val> DOB BW
 *   c<amp>  manual current (disabled only, polarity check)
 *   z   set origin (pos->0)
 */

#include <FlexCAN_T4.h>

// ==================== PER-MOTOR PARAMETERS ====================
// Kt = 1.5 * POLE_PAIRS * lambda[Wb] (measured). DOB_BW < current-loop BW(=Kp/L) & < ctrl rate.
// J_nom: NOT from electrical meas — datasheet rotor inertia (x gear^2) or tune.
#define M1_CAN_ID 1
#define M1_KT     0.0705f   // shoulder: 1.5*14*3.356mWb
#define M1_J_NOM  0.0015f
#define M1_DOB_BW 80.0f
#define M1_DIR    +1.0f
#define M2_CAN_ID 2
#define M2_KT     0.0908f   // elbow: 1.5*14*4.324mWb
#define M2_J_NOM  0.0015f
#define M2_DOB_BW 80.0f
#define M2_DIR    +1.0f

// --- Mechanics (common AK40-10) ---
#define GEAR_RATIO 10.0f
#define GEAR_EFF   0.90f
#define POLE_PAIRS 14
#define POS_AT_JOINT 1       // pos joint-side; spd is motor ERPM -> /pp/gear

// --- Defaults (runtime-adjustable) ---
#define K_IMP_INIT  2.0f
#define D_IMP_INIT  0.05f
#define OMEGA_LPF_HZ 25.0f
#define I_MAX        2.0f
#define POS_ERR_LIMIT_DEG 60.0f
#define CTRL_HZ      200
// =============================================================

#define STATUS_PKT 0x29
#define DEG2RAD   0.0174532925f
#define RPM2RADS  0.1047197551f
#define NUM_MOTORS 2

typedef enum {
  CAN_PACKET_SET_DUTY        = 0,
  CAN_PACKET_SET_CURRENT     = 1,
  CAN_PACKET_SET_ORIGIN_HERE = 5,
} CAN_PACKET_ID;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

struct Motor {
  uint8_t can_id;
  float Kt, J_nom, dob_bw, dir;
  float theta, omega;          // measured (joint frame)
  bool  haveStatus;
  float theta_d, omega_d;
  float K, D;
  bool  enabled, dob_on;
  float omega_f, dob_w, dob_v, tau_applied;
  float dbg_tau_d, dbg_dhat, dbg_I;
};
Motor mot[NUM_MOTORS];

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
  for (int k = 0; k < NUM_MOTORS; k++) {
    if (mot[k].can_id != id) continue;
    float pos_deg = buf_get_int16(msg.buf, 0) * 0.1f;
    float erpm    = buf_get_int16(msg.buf, 2) * 10.0f;
    if (!POS_AT_JOINT) pos_deg /= GEAR_RATIO;
    mot[k].theta = pos_deg * DEG2RAD;
    mot[k].omega = (erpm / (float)POLE_PAIRS / GEAR_RATIO) * RPM2RADS;
    mot[k].haveStatus = true;
    return;
  }
}

static float dobUpdate(Motor &m, float tau, float om, float dt) {
  m.dob_w += dt * m.dob_bw * (om  - m.dob_w);
  m.dob_v += dt * m.dob_bw * (tau - m.dob_v);
  return m.J_nom * m.dob_bw * (om - m.dob_w) - m.dob_v;
}

static void disableMotor(Motor &m) {
  m.enabled = false;
  m.tau_applied = 0; m.dob_w = 0; m.dob_v = 0; m.omega_f = 0;
  m.dbg_tau_d = 0; m.dbg_dhat = 0; m.dbg_I = 0;
  motorSetCurrent(m.can_id, 0.0f);
}

static void controlStep(Motor &m, float dt) {
  if (!m.enabled || !m.haveStatus) return;
  float th = m.theta, om = m.omega;

  if (fabsf(m.theta_d - th) > POS_ERR_LIMIT_DEG * DEG2RAD) {
    disableMotor(m);
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

static void applyEnable(Motor &m, bool dob) {
  m.theta_d = m.theta;
  m.dob_w = 0; m.dob_v = 0; m.tau_applied = 0; m.omega_f = 0;
  m.dob_on = dob; m.enabled = true;
  Serial.printf(">> M%u ENABLED %s, target=%.1f deg\n",
                m.can_id, dob ? "(imp+DOB)" : "(imp)", m.theta_d / DEG2RAD);
}

// command target: -1 = all motors, else single index
int activeIdx = 0;
static inline bool isTarget(int i) { return activeIdx < 0 || activeIdx == i; }

String rxLine;
void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s == "m1") { activeIdx = 0;  Serial.println(">> target M1");  return; }
  if (s == "m2") { activeIdx = 1;  Serial.println(">> target M2");  return; }
  if (s == "ma") { activeIdx = -1; Serial.println(">> target ALL"); return; }

  if (s == "e" || s == "d") {
    for (int i = 0; i < NUM_MOTORS; i++) if (isTarget(i)) applyEnable(mot[i], s == "d");
    return;
  }
  if (s == "s") {
    for (int i = 0; i < NUM_MOTORS; i++) if (isTarget(i)) disableMotor(mot[i]);
    Serial.println(">> STOP"); return;
  }
  if (s == "o") {
    for (int i = 0; i < NUM_MOTORS; i++) if (isTarget(i)) {
      mot[i].dob_on = !mot[i].dob_on; mot[i].dob_w = 0; mot[i].dob_v = 0;
      Serial.printf(">> M%u DOB %s\n", mot[i].can_id, mot[i].dob_on ? "ON" : "OFF");
    }
    return;
  }
  if (s == "z") {
    for (int i = 0; i < NUM_MOTORS; i++) if (isTarget(i)) {
      if (mot[i].enabled) Serial.printf("M%u: disable first\n", mot[i].can_id);
      else { motorSetOrigin(mot[i].can_id); Serial.printf(">> M%u origin set\n", mot[i].can_id); }
    }
    return;
  }

  if (s.length() < 2) return;
  char c = s.charAt(0);
  float v = s.substring(1).toFloat();
  switch (c) {
    case 't': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) mot[i].theta_d = v*DEG2RAD;
              Serial.printf(">> target %.1f deg\n", v); break;
    case 'k': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) mot[i].K = v;
              Serial.printf(">> K=%.3f\n", v); break;
    case 'b': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) mot[i].D = v;
              Serial.printf(">> D=%.3f\n", v); break;
    case 'j': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) mot[i].J_nom = v;
              Serial.printf(">> DOB J=%.5f\n", v); break;
    case 'g': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) mot[i].dob_bw = v;
              Serial.printf(">> DOB BW=%.1f\n", v); break;
    case 'c': for (int i=0;i<NUM_MOTORS;i++) if(isTarget(i)) {
                if (mot[i].enabled) Serial.printf("M%u: disable first\n", mot[i].can_id);
                else { motorSetCurrent(mot[i].can_id, v);
                       Serial.printf(">> M%u %.2fA pulse\n", mot[i].can_id, v); }
              } break;
    default: Serial.printf("?? unknown: '%s'\n", s.c_str());
  }
}

uint32_t lastCtrl = 0, lastPrint = 0;
const uint32_t CTRL_US = 1000000UL / CTRL_HZ;

static void initMotor(Motor &m, uint8_t id, float kt, float jn, float bw, float dir) {
  m = Motor{};
  m.can_id = id; m.Kt = kt; m.J_nom = jn; m.dob_bw = bw; m.dir = dir;
  m.K = K_IMP_INIT; m.D = D_IMP_INIT;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  initMotor(mot[0], M1_CAN_ID, M1_KT, M1_J_NOM, M1_DOB_BW, M1_DIR);
  initMotor(mot[1], M2_CAN_ID, M2_KT, M2_J_NOM, M2_DOB_BW, M2_DIR);

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.setFIFOFilter(ACCEPT_ALL);

  lastCtrl = micros();
  Serial.println("AK40 x2 impedance+DOB. m1/m2/ma  e/d/s/o  t k b j g c z");
}

void loop() {
  CAN_message_t rx;
  while (Can1.read(rx)) parseStatus(rx);

  uint32_t now = micros();
  if ((uint32_t)(now - lastCtrl) >= CTRL_US) {
    float dt = (now - lastCtrl) * 1e-6f;
    lastCtrl = now;
    for (int i = 0; i < NUM_MOTORS; i++) controlStep(mot[i], dt);
  }

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    for (int i = 0; i < NUM_MOTORS; i++) {
      Motor &m = mot[i];
      bool active = (fabsf(m.omega) > 0.3f) || (fabsf(m.dbg_I) > 0.05f);
      if (m.haveStatus && active) {
        Serial.printf("M%u th=%.1f w=%.2f | tau_d=%.3f dhat=%.3f I=%.2f %s\n",
                      m.can_id, m.theta / DEG2RAD, m.omega,
                      m.dbg_tau_d, m.dbg_dhat, m.dbg_I,
                      !m.enabled ? "off" : (m.dob_on ? "ON+DOB" : "ON"));
      }
    }
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length()) { handleLine(rxLine); rxLine = ""; }
    } else if (rxLine.length() < 64) rxLine += c;
  }
}
