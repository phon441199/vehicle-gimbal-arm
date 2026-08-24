// =============================================================================
//  이중 서보 조화가진장치 — IMU 기반 위상동기 제어  (ESP32)
//
//  - 서보 2개(DS3345, 연속회전)가 크랭크-슬라이더로 판을 상하 가진(약 2 Hz)
//  - 마스터(GPIO18)는 기준속도 u0로 개루프 구동, 슬레이브(GPIO19)는 PI로 보정
//  - IMU(MPU-6500, I2C 21/22)로 판의 롤을 측정 → 상관검파로 위상오차 추출
//  - PI 적분기가 위상오차를 0으로 몰아 두 서보 평균속도를 맞춤(드리프트 제거)
//
//  시리얼(115200) 명령:
//    s 정지 / c 정방향 / a 역방향
//    y 동기보정 토글 / d 디버그출력 토글 / k 게인출력 / f 보정부호 반전
//    1·2 = Kp -/+    3·4 = Ki -/+    m = I2C스캔    i = IMU진단
//    v<int> = 가진속도(=주파수) 설정 0..987 (~offset/200 Hz, 예: v600 -> ~3Hz)
//  명령은 USB serial 또는 WiFi TCP(포트 3333) 동일하게 받음. 전원: 5V/VIN핀(USB 불요).
//
//  배선: 서보1->GPIO18, 서보2->GPIO19, IMU SDA->21 SCL->22 (3.3V, GND공통)
//  의존성: madhephaestus/ESP32Servo  (platformio.ini 참조)
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>

// ====================== WiFi / TCP remote (USB-free) ======================
// Power the ESP32 from 5V on the 5V/VIN pin (servo supply OK, common GND) -- no USB needed.
// Commands arrive over a TCP socket exactly like serial (same single chars); replies go back
// to the sender. Serial stays active as a fallback. ESP32 WiFi is 2.4GHz only.
// Switch to AP (ESP hotspot, fixed IP 192.168.4.1) later by setting USE_AP 1.
#define USE_WIFI      0                 // 0 = WIRED (USB serial) ONLY -- skips WiFi init, no boot delay. 1 = enable WiFi/TCP.
#define USE_AP        0                 // (only when USE_WIFI=1) 0 = STA join router | 1 = AP hotspot
#define WIFI_SSID     "YOUR_WIFI"       // <-- FILL IN (STA): 2.4GHz router SSID
#define WIFI_PASS     "YOUR_PASS"       // <-- FILL IN (STA): router password
#define AP_SSID       "shaker"          // AP-mode SSID  (used only when USE_AP=1)
#define AP_PASS       "shaker1234"      // AP-mode password (>=8 chars)
#define TCP_PORT      3333
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// ====================== 서보 설정 ======================
static constexpr int SERVO_PIN1 = 18;
static constexpr int SERVO_PIN2 = 19;
static constexpr int STOP_PULSE = 1513;  // DS3345 데드밴드(1504~1522)의 중앙값

// 마주 보고 설치 → 같은 명령에 물리적으로 반대로 돌아야 하므로 회전 부호를 다르게 둠
// dirSign = +1 이면 (STOP+offset) 방향, -1 이면 (STOP-offset) 방향
static constexpr int DIR_SIGN1 = -1;  // 서보1(18, 기준)
static constexpr int DIR_SIGN2 = +1;  // 서보2(19): 마주보기라 반대 부호

// 연속 가진 속도(정지값으로부터의 펄스 오프셋). 최대치(987)가 아니라 중간값으로!
// → 슬레이브를 빠르게/느리게 양방향으로 trim 할 여유를 남기기 위함.
static int driveOffset = 400;             // runtime drive speed (was const BASE_OFFSET) -- sets excitation
                                          // freq (~offset/200 Hz, 0~987). change live via 'v<int>'
static constexpr int TRIM_LIMIT = 250;    // 슬레이브 보정 최대 폭

Servo myServo1;
Servo myServo2;

// ====================== 위상동기(IMU) 제어 게인 (런타임 조정) ======================
// 시리얼로 실시간 튜닝: f=부호반전, 1/2=KP-/+, 3/4=KI-/+, k=현재값 출력
float corrSign = +1.0f;
// 적분 위주 설계: 느린 위상 드리프트만 잡고, 매 사이클 외란은 주입하지 않음
float Kp = 8.0f;      // 비례 (작게 — 리플 외란 방지)
float Ki = 40.0f;     // 적분 (평균속도 정합 = 드리프트 제거 핵심)
static constexpr float HP_ALPHA = 0.02f;    // 가속/자이로 DC 제거용 저역통과 계수
static constexpr float ERR_ALPHA = 0.006f;  // 상관검파 강한 평활화 (DC 위상오차만 추출, 리플 제거)

// ====================== MPU9250/9265 최소 드라이버 ======================
static constexpr uint8_t MPU_ADDR = 0x68;  // AD0=GND 기준 (0x68)
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_WHO_AM_I = 0x75;

// 어느 축이 "수직 가속"이고 어느 축이 "두 크랭크를 잇는 선에 대한 롤레이트"인지
// 장착 방향에 따라 다름. 'd' 디버그로 확인 후 아래 인덱스/부호를 맞추세요.
// 인덱스: 0=X, 1=Y, 2=Z
static constexpr int VERT_ACC_AXIS = 2;   // 수직 가속도 = accel Z (실측 확인: 중력 1.0g)
static constexpr int ROLL_GYR_AXIS = 1;   // 롤레이트 = gyro Y (실측: 운전 중 진동 최대)

float accel_g[3];   // g 단위
float gyro_dps[3];  // deg/s 단위

bool mpuOK = false;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);   // STOP (반복시작 대신 — 약한 풀업/마진 대응)
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

// 멈춘(wedged) I2C 슬레이브를 SCL 9펄스로 강제 해제하고 STOP 발생
void i2cBusRecover(int sdaPin, int sclPin) {
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);  delayMicroseconds(10);
    digitalWrite(sclPin, HIGH); delayMicroseconds(10);
  }
  // STOP 조건: SDA가 SCL HIGH 동안 LOW→HIGH
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH); delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(10);
}

bool mpuBegin() {
  i2cBusRecover(21, 22);   // 멈춘 슬레이브 해제
  Wire.begin(21, 22);      // SDA=21, SCL=22 명시
  Wire.setClock(100000);   // 100kHz (긴 배선 고려한 안전값)
  Wire.setTimeOut(250);    // 버스 타임아웃 여유

  // 정식 리셋 시퀀스 (MPU6500/9250 호환)
  mpuWrite(REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET
  delay(100);
  mpuWrite(REG_PWR_MGMT_1, 0x01);  // clock = auto PLL, sleep 해제
  delay(10);
  mpuWrite(REG_PWR_MGMT_2, 0x00);  // accel+gyro 전 축 enable
  delay(10);
  mpuWrite(REG_GYRO_CONFIG, 0x00);   // ±250 dps
  mpuWrite(REG_ACCEL_CONFIG, 0x00);  // ±2 g
  delay(10);

  uint8_t who = mpuRead8(REG_WHO_AM_I);
  // MPU9250=0x71, MPU9255=0x73, MPU6500=0x70, 일부 클론=0x68 등
  Serial.printf("WHO_AM_I = 0x%02X\n", who);
  return (who != 0x00 && who != 0xFF);
}

void mpuReadAll() {
  // 최대 3회 재시도 (마진/약한 풀업 대응), STOP 방식 사용
  int got = 0;
  for (int tries = 0; tries < 3 && got < 14; tries++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(true);
    got = Wire.requestFrom((int)MPU_ADDR, 14);
    if (got < 14) { while (Wire.available()) Wire.read(); delayMicroseconds(200); }
  }
  if (got < 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temp 버림
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // 기본 감도: 가속 ±2g(16384 LSB/g), 자이로 ±250dps(131 LSB/dps)
  accel_g[0] = ax / 16384.0f;
  accel_g[1] = ay / 16384.0f;
  accel_g[2] = az / 16384.0f;
  gyro_dps[0] = gx / 131.0f;
  gyro_dps[1] = gy / 131.0f;
  gyro_dps[2] = gz / 131.0f;
}

// ====================== 상태 ======================
int currentMode = 0;       // 0=stop, 1=정방향, 2=역방향
bool syncEnabled = true;    // 위상동기 보정 on/off
bool debugPrint = false;

// 필터 상태
float accLP = 0, gyrLP = 0;  // DC(중력/바이어스) 추정
float errFilt = 0;           // 상관 검파 결과(위상오차 추정)
float integ = 0;             // 적분기
float trim = 0;              // 슬레이브(서보2) 보정 오프셋
uint32_t lastUs = 0;

void writeServos(int offset1, int offset2) {
  int p1 = STOP_PULSE + DIR_SIGN1 * offset1;
  int p2 = STOP_PULSE + DIR_SIGN2 * offset2;
  myServo1.writeMicroseconds(constrain(p1, 500, 2500));
  myServo2.writeMicroseconds(constrain(p2, 500, 2500));
}

void applyStop() {
  myServo1.writeMicroseconds(STOP_PULSE);
  myServo2.writeMicroseconds(STOP_PULSE);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);   // bound Serial.parseInt() for the 'v<int>' freq command
  delay(1000);

  myServo1.setPeriodHertz(50);
  myServo1.attach(SERVO_PIN1, 500, 2500);
  myServo2.setPeriodHertz(50);
  myServo2.attach(SERVO_PIN2, 500, 2500);
  applyStop();

  mpuOK = mpuBegin();
  Serial.println(mpuOK ? "MPU init OK" : "MPU init FAILED (check wiring/addr)");

  Serial.println("=== Harmonic exciter w/ IMU phase-sync ===");
  Serial.println("Commands:");
  Serial.println("  s - stop");
  Serial.println("  c - run forward");
  Serial.println("  a - run reverse");
  Serial.println("  y - toggle sync correction");
  Serial.println("  d - toggle debug print");

  // ---- WiFi + TCP server (USB-free remote) ----
#if USE_WIFI
#if USE_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi AP '%s'  IP %s  TCP %d\n", AP_SSID, WiFi.softAPIP().toString().c_str(), TCP_PORT);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi(STA) connecting");
  uint32_t wt0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wt0 < 10000) { delay(250); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\nWiFi OK  IP %s  TCP %d  <- point the GUI here\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
  else
    Serial.println("\nWiFi FAILED (check SSID/PASS; serial still works)");
#endif
  tcpServer.begin();
  tcpServer.setNoDelay(true);
#endif  // USE_WIFI

  lastUs = micros();
}

// io = the transport the command came from (Serial or a WiFiClient) -- replies + the 'v'
// number are read/written there, so serial and WiFi behave identically.
void handleCommand(char cmd, Stream &io) {
  if (cmd == 's' || cmd == 'S') {
    currentMode = 0;
    integ = 0; trim = 0;        // 정지 시 적분기 리셋
    io.println("Stopped");
  } else if (cmd == 'c' || cmd == 'C') {
    currentMode = 1;
    io.println("Run forward");
  } else if (cmd == 'a' || cmd == 'A') {
    currentMode = 2;
    io.println("Run reverse");
  } else if (cmd == 'y' || cmd == 'Y') {
    syncEnabled = !syncEnabled;
    integ = 0; trim = 0;
    io.printf("Sync %s\n", syncEnabled ? "ON" : "OFF");
  } else if (cmd == 'd' || cmd == 'D') {
    debugPrint = !debugPrint;
    io.printf("Debug %s\n", debugPrint ? "ON" : "OFF");
  } else if (cmd == 'f' || cmd == 'F') {
    corrSign = -corrSign;
    integ = 0; trim = 0;
    io.printf("CORR_SIGN = %+.0f\n", corrSign);
  } else if (cmd == '1') { Kp = max(0.0f, Kp - 5); io.printf("Kp=%.1f\n", Kp);
  } else if (cmd == '2') { Kp += 5; io.printf("Kp=%.1f\n", Kp);
  } else if (cmd == '3') { Ki = max(0.0f, Ki - 5); io.printf("Ki=%.1f\n", Ki);
  } else if (cmd == '4') { Ki += 5; io.printf("Ki=%.1f\n", Ki);
  } else if (cmd == 'k' || cmd == 'K') {
    io.printf("Kp=%.1f Ki=%.1f sign=%+.0f sync=%d off=%d (~%.2fHz)\n",
              Kp, Ki, corrSign, syncEnabled, driveOffset, driveOffset / 200.0f);
  } else if (cmd == 'v' || cmd == 'V') {       // set drive speed = excitation freq. sender sends "v<int>\n"
    long val = io.parseInt();                  // offset 0..987 (~offset/200 Hz) -- read from same transport
    driveOffset = constrain((int)val, 0, 987);
    io.printf("driveOffset=%d (~%.2f Hz)\n", driveOffset, driveOffset / 200.0f);
  } else if (cmd == 'm' || cmd == 'M') {
    // I2C 버스 스캔: 어떤 주소가 ACK 하는지 확인
    io.println("I2C scan...");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        io.printf("  device at 0x%02X\n", a);
        found++;
      }
    }
    io.printf("scan done, %d device(s)\n", found);
  } else if (cmd == 'i' || cmd == 'I') {
    // IMU 진단: WHO_AM_I, 수신 바이트 수, 원시 값 출력
    uint8_t who = mpuRead8(REG_WHO_AM_I);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(true);
    int n = Wire.requestFrom((int)MPU_ADDR, 14);
    io.printf("WHO=0x%02X bytes=%d\n", who, n);
    mpuReadAll();
    io.printf("accel(g)=%.3f, %.3f, %.3f  gyro(dps)=%.2f, %.2f, %.2f\n",
              accel_g[0], accel_g[1], accel_g[2],
              gyro_dps[0], gyro_dps[1], gyro_dps[2]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand((char)Serial.read(), Serial);
  }
#if USE_WIFI
  // WiFi TCP client: keep one connected; feed its bytes to the same handler (replies go back to it)
  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.available();
    if (tcpClient) tcpClient.setTimeout(50);
  }
  if (tcpClient && tcpClient.available() > 0) {
    handleCommand((char)tcpClient.read(), tcpClient);
  }
#endif

  // ---- 고정 샘플 주기(약 5ms = 200Hz) ----
  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  if (dt < 0.005f) return;
  lastUs = now;

  if (mpuOK) mpuReadAll();

  // ---- 상관 검파로 위상오차 추정 (드라이브 주파수 불필요) ----
  // 원리: <롤레이트 × 수직가속도> 의 시간평균 ∝ sin(위상차)
  float accV = accel_g[VERT_ACC_AXIS];
  float gyrR = gyro_dps[ROLL_GYR_AXIS];

  // DC(중력/바이어스) 제거 → AC 성분만 남김
  accLP += HP_ALPHA * (accV - accLP);
  gyrLP += HP_ALPHA * (gyrR - gyrLP);
  float accAC = accV - accLP;
  float gyrAC = gyrR - gyrLP;

  float inst = gyrAC * accAC;            // 순간 상관
  errFilt += ERR_ALPHA * (inst - errFilt);  // 평활화 → 위상오차 추정

  // ---- 운전 중일 때만 보정 ----
  if (currentMode == 0) {
    applyStop();
    if (debugPrint) {
      // 정지 중에도 IMU 축 확인용으로 값 출력 (손으로 판을 기울여 반응 확인)
      Serial.printf("STOP accAC=%.3f gyrAC=%.2f (accV=%.2f gyrR=%.2f)\n",
                    accAC, gyrAC, accV, gyrR);
    }
    return;
  }

  float error = corrSign * errFilt;

  if (syncEnabled) {
    integ += Ki * error * dt;
    integ = constrain(integ, -(float)TRIM_LIMIT, (float)TRIM_LIMIT);
    trim = Kp * error + integ;
    trim = constrain(trim, -(float)TRIM_LIMIT, (float)TRIM_LIMIT);
  } else {
    trim = 0;
  }

  // 정방향(1)/역방향(2)에 따라 두 모터의 진행 부호 결정
  int dir = (currentMode == 1) ? +1 : -1;
  int off1 = dir * driveOffset;
  int off2 = dir * (driveOffset + (int)lroundf(trim));
  off1 = constrain(off1, -987, 987);
  off2 = constrain(off2, -987, 987);

  // dir 부호는 writeServos 내부 DIR_SIGN 과 곱해져 최종 방향이 됨
  writeServos(off1, off2);

  if (debugPrint) {
    Serial.printf("accAC=%.3f gyrAC=%.2f err=%.4f trim=%.1f off2=%d\n",
                  accAC, gyrAC, errFilt, trim, off2);
  }
}
