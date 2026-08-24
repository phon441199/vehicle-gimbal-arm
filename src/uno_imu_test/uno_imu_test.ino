/*
 * uno_imu_test.ino  -- isolate the ICM-20948 on a bench ESP8266 NodeMCU (SPI).
 * Purpose: prove whether the IMU CHIP is alive, away from the 90cm shared bus.
 *
 * ESP8266 is 3.3V logic -- SAFE for the ICM-20948, no level shifting needed.
 *
 * Wiring (NodeMCU hardware SPI; D-label -> GPIO):
 *   IMU SCK  <- D5 (GPIO14)
 *   IMU MISO -> D6 (GPIO12)
 *   IMU MOSI <- D7 (GPIO13)
 *   IMU CS   <- D1 (GPIO5)    -- avoid D8/GPIO15 (boot strap)
 *   IMU 3V3  <- 3V3
 *   IMU GND  <- GND
 *
 * Board: "NodeMCU 1.0 (ESP-12E Module)".  Library: SparkFun ICM_20948.
 * Reads WHO_AM_I at begin() -> "CHIP ALIVE" if it answers.  Then streams accel/gyro
 * so you can confirm the data changes when you move it.
 */
#include <SPI.h>
#include "ICM_20948.h"

#define CS_PIN 5            // GPIO5 = NodeMCU D1 (SPI SCK/MISO/MOSI = GPIO14/12/13 = D5/D6/D7)

ICM_20948_SPI myICM;
bool imu_ok = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);
  Serial.println("\n=== ICM-20948 bench SPI test ===");
  SPI.begin();

  for (int i = 0; i < 5; i++) {
    myICM.begin(CS_PIN, SPI);                 // reads WHO_AM_I (expect 0xEA)
    Serial.print("begin try "); Serial.print(i + 1);
    Serial.print(": "); Serial.println(myICM.statusString());
    if (myICM.status == ICM_20948_Stat_Ok) { imu_ok = true; break; }
    delay(500);
  }
  Serial.println(imu_ok ? ">> CHIP ALIVE (WHO_AM_I ok)"
                        : ">> CHIP FAIL (no response -- wiring or dead chip)");
}

void loop() {
  if (!imu_ok) {                              // keep retrying so you can wiggle wires
    myICM.begin(CS_PIN, SPI);
    if (myICM.status == ICM_20948_Stat_Ok) { imu_ok = true; Serial.println(">> recovered: CHIP ALIVE"); }
    delay(500);
    return;
  }
  myICM.getAGMT();                            // no dataReady gate (read registers directly)
  Serial.print("ax="); Serial.print(myICM.accX(), 0);
  Serial.print(" ay="); Serial.print(myICM.accY(), 0);
  Serial.print(" az="); Serial.print(myICM.accZ(), 0);
  Serial.print(" | gz="); Serial.print(myICM.gyrZ(), 1);
  Serial.print(" gx="); Serial.println(myICM.gyrX(), 1);
  delay(200);                                 // move the board -> values should change
}
