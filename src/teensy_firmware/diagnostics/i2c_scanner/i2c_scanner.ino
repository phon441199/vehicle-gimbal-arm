/*
 * IMU Scanner for Teensy 4.1 -- joint IMUs (I2C) + wrist IMU (SPI).
 *
 *   I2C joint MPU6050s:
 *     Wire  : SDA = 18, SCL = 19   (shoulder MPU6050)
 *     Wire2 : SDA = 25, SCL = 24   (elbow MPU6050)
 *     Speed = 100 kHz (tolerant). drop to 10000 if long/weak-pullup cable.
 *     MPU6050 expected: AD0=GND -> 0x68,  AD0=3V3 -> 0x69.
 *
 *   SPI wrist IMU ICM20948 (the one the I2C scan does NOT see -- it is on SPI):
 *     SPI0 : SCK = 13, MISO = 12, MOSI = 11,  CS = 28   (shared bus w/ enc CS 9,29)
 *     WHO_AM_I (bank0 reg 0x00) = 0xEA when alive.
 *     Mode is uncertain across libs/clones -> we probe SPI_MODE0 and SPI_MODE3 both.
 */

#include <Wire.h>
#include <SPI.h>

// ---- SPI wrist IMU pins (match arm_microros_teensy) ----
#define WR_IMU_CS     28
#define WR_ENC_CS      9
#define WR_ENC2_CS    29
#define WR_IMU_SPI_HZ 1000000UL
#define ICM_WHO_AM_I  0x00      // bank 0
#define ICM_BANK_SEL  0x7F
#define ICM_WHOAMI_OK 0xEA

static void scan(TwoWire &w, const char *name) {
  byte count = 0;
  Serial.print("--- "); Serial.print(name); Serial.println(" ---");
  for (byte addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    if (w.endTransmission() == 0) {
      Serial.print("  found 0x"); if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices -- check power/SDA-SCL/pullups)");
}

// ---- raw SPI register access for ICM20948 (read bit = MSB of addr) ----
static uint8_t icm_read(uint8_t reg, uint8_t mode) {
  SPI.beginTransaction(SPISettings(WR_IMU_SPI_HZ, MSBFIRST, mode));
  digitalWrite(WR_IMU_CS, LOW);
  SPI.transfer(reg | 0x80);             // 0x80 = read
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(WR_IMU_CS, HIGH);
  SPI.endTransaction();
  return v;
}
static void icm_write(uint8_t reg, uint8_t val, uint8_t mode) {
  SPI.beginTransaction(SPISettings(WR_IMU_SPI_HZ, MSBFIRST, mode));
  digitalWrite(WR_IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);             // clear MSB = write
  SPI.transfer(val);
  digitalWrite(WR_IMU_CS, HIGH);
  SPI.endTransaction();
}

static void scan_wrist_imu() {
  Serial.println("--- wrist IMU ICM20948 (SPI0 SCK13/MISO12/MOSI11, CS28) ---");
  const uint8_t mode_v[2] = { SPI_MODE0, SPI_MODE3 };
  const char   *mode_n[2] = { "MODE0",  "MODE3"  };
  bool found = false;
  for (int i = 0; i < 2; i++) {
    icm_write(ICM_BANK_SEL, 0x00, mode_v[i]);     // select user bank 0
    uint8_t who = icm_read(ICM_WHO_AM_I, mode_v[i]);
    Serial.print("  SPI_"); Serial.print(mode_n[i]);
    Serial.print(": WHO_AM_I=0x"); if (who < 16) Serial.print("0"); Serial.print(who, HEX);
    if (who == ICM_WHOAMI_OK) { Serial.println("  <-- ICM20948 FOUND"); found = true; }
    else                        Serial.println();
  }
  if (!found)
    Serial.println("  (not found: expect 0xEA. 0x00/0xFF = dead bus -> check CS28/power/SCK-MISO-MOSI/loose header)");
}

void setup() {
  Wire.begin();  Wire.setClock(100000);
  Wire2.begin(); Wire2.setClock(100000);
  // SPI: park every shared CS HIGH (a floating CS drives MISO and corrupts the probe)
  pinMode(WR_IMU_CS,  OUTPUT); digitalWrite(WR_IMU_CS,  HIGH);
  pinMode(WR_ENC_CS,  OUTPUT); digitalWrite(WR_ENC_CS,  HIGH);
  pinMode(WR_ENC2_CS, OUTPUT); digitalWrite(WR_ENC2_CS, HIGH);
  SPI.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("IMU Scanner -- I2C Wire(18/19)+Wire2(25/24) joints + SPI ICM20948(CS28) wrist");
}

void loop() {
  scan(Wire,  "Wire  (SDA18 SCL19) = shoulder MPU6050");
  scan(Wire2, "Wire2 (SDA25 SCL24) = elbow MPU6050");
  scan_wrist_imu();
  Serial.println();
  delay(2000);
}
