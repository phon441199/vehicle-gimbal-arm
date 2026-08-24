/*
 * spi_miso_probe.ino  -- LOLIN D32.  Probe enc1(CS21)/enc2(CS22)/IMU(CS13) using the SAME
 * read path as the real firmware (SimpleFOC MagneticSensorSPI + SparkFun ICM_20948), so the
 * results are MEANINGFUL.  (A raw transfer16 without sensor.init() returns 0 -- useless.)
 *
 *   enc1: rotate the shaft -> getAngle() should change.  This is the known-good reference.
 *   enc2: getAngle() varies when rotated -> OK ; frozen/0 -> not reading
 *   IMU : "begin OK" + accel value -> chip responds on the D32 bus ; "FAIL" -> no WHO_AM_I
 *
 * Bus clock 250k (matches the lowered firmware).  All CS parked HIGH first.
 */
#include <SimpleFOC.h>
#include <SPI.h>
#include <ICM_20948.h>

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23

MagneticSensorSPIConfig_s AS5048_CFG = {
  .spi_mode = SPI_MODE1, .clock_speed = 250000, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI enc1 = MagneticSensorSPI(AS5048_CFG, 21);
MagneticSensorSPI enc2 = MagneticSensorSPI(AS5048_CFG, 15);  // moved off damaged GPIO22 (shorted to MOSI/23)
ICM_20948_SPI imu;
bool imu_ok = false;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SPI probe (real read path) ===");
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  pinMode(21, OUTPUT); digitalWrite(21, HIGH);   // enc1 CS
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);   // enc2 CS now on GPIO15
  pinMode(13, OUTPUT); digitalWrite(13, HIGH);   // IMU CS
  pinMode(22, INPUT);                            // damaged GPIO22: leave UNDRIVEN so it can't fight MOSI(23)
  // original order restored: encoders first, IMU last
  enc1.init(&SPI);
  enc2.init(&SPI);
  for (int i = 0; i < 5; i++) {
    imu.begin(13, SPI, 250000);
    if (imu.status == ICM_20948_Stat_Ok) { imu_ok = true; break; }
    delay(200);
  }
  Serial.print("IMU begin: "); Serial.println(imu_ok ? "OK (chip responds)" : "FAIL (no WHO_AM_I)");
}

void loop() {
  enc1.update(); enc2.update();
  Serial.print("enc1="); Serial.print(enc1.getAngle(), 3);
  Serial.print("  enc2="); Serial.print(enc2.getAngle(), 3);
  if (imu_ok) { imu.getAGMT();
    Serial.print("  imu ax="); Serial.print(imu.accX(), 0);
    Serial.print(" ay=");      Serial.print(imu.accY(), 0);
  } else Serial.print("  imu=FAIL");
  Serial.println();
  delay(300);                 // rotate enc1/enc2 shafts -> their angles should change
}
