/**
 * enc_imu_spi_test.ino  -  LOLIN D32 (ESP32)
 * Minimal bring-up: 2x AS5048A encoders + ICM20948 IMU on ONE shared SPI bus.
 * No motors, no CAN. Use this to verify the 70cm LAN run before full firmware.
 *
 * ---- WIRING (shared SPI bus over Cat5 to wrist) ----------------------------
 *   SCK  = 18   (LAN orange)        MISO = 19   (LAN blue)
 *   MOSI = 23   (LAN green)         GND  = (LAN white-orange)
 *   ENC1_CS = 21 (LAN white-blue)   3V3  = (LAN brown)
 *   ENC2_CS = 22 (LAN white-brown)
 *   IMU_CS  = 13 (LAN white-green)
 *   ICM20948: SCL=SCK SDA=MOSI AD0=MISO NCS=IMU_CS  (FSYNC->GND, INT/AUX open)
 *   Star ground: motor power GND must NOT share this LAN GND.
 * ----------------------------------------------------------------------------
 *
 * Both encoder (SimpleFOC) and IMU (SparkFun) wrap each transfer in their own
 * beginTransaction(), so SPI mode (AS5048=MODE1, ICM=internal) switches per
 * device automatically on the shared bus. Clock kept low for the long cable.
 *
 * Serial @115200. Keys: e=enc only  i=imu only  b=both(default)  x=stop stream
 *
 * Libraries: SimpleFOC, SparkFun ICM_20948
 */

#include <SimpleFOC.h>
#include <SPI.h>
#include <ICM_20948.h>

// ---- pins (shared SPI) -----------------------------------------------------
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define ENC1_CS   21
#define ENC2_CS   22
#define IMU_CS    13

// ---- bus speeds (low for 70cm cable) ---------------------------------------
#define ENC_SPI_HZ  1000000UL   // 1 MHz, raise later if clean
#define IMU_SPI_HZ  2000000UL   // 2 MHz

// AS5048A 14-bit SPI config (mode1)
MagneticSensorSPIConfig_s AS5048_CFG = {
  .spi_mode = SPI_MODE1, .clock_speed = ENC_SPI_HZ, .bit_resolution = 14,
  .angle_register = 0x3FFF, .data_start_bit = 13,
  .command_rw_bit = 14, .command_parity_bit = 15
};
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5048_CFG, ENC1_CS);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5048_CFG, ENC2_CS);

ICM_20948_SPI imu;
bool imu_ok = false;

enum StreamMode { BOTH, ENC_ONLY, IMU_ONLY, STOP };
StreamMode stream_mode = BOTH;
unsigned long last_ms = 0;

// raw register poke to confirm an encoder actually answers (not just 0xFFFF)
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

bool setup_imu() {
  imu.begin(IMU_CS, SPI, IMU_SPI_HZ);
  if (imu.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fss; fss.a = gpm4; fss.g = dps500;
  imu.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_dlpcfg_t dlp; dlp.a = acc_d50bw4_n68bw8; dlp.g = gyr_d51bw2_n73bw3;
  imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  return (imu.status == ICM_20948_Stat_Ok);
}

void print_status() {
  bool e1 = encoder_alive(ENC1_CS);
  bool e2 = encoder_alive(ENC2_CS);
  Serial.printf("\n--- status ---  ENC1=%s  ENC2=%s  IMU=%s\n",
    e1 ? "ALIVE" : "NO RESP", e2 ? "ALIVE" : "NO RESP",
    imu_ok ? "OK" : "FAIL");
  Serial.println("keys: e=enc  i=imu  b=both  s=status  x=stop");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n### enc+imu SPI shared-bus test (ESP32/D32) ###");

  // explicit pins; manual CS (SS = -1). Default VSPI happens to match 18/19/23.
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
  pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);
  pinMode(IMU_CS,  OUTPUT); digitalWrite(IMU_CS,  HIGH);

  encoder1.init(&SPI);
  encoder2.init(&SPI);

  Serial.print("IMU init... ");
  for (int i = 0; i < 10; i++) { if (setup_imu()) { imu_ok = true; break; } delay(200); }
  Serial.println(imu_ok ? "OK" : "FAILED");

  print_status();
}

void loop() {
  // keep encoder angle tracking fresh
  encoder1.update();
  encoder2.update();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') { stream_mode = ENC_ONLY; Serial.println(">> enc"); }
    else if (c == 'i') { stream_mode = IMU_ONLY; Serial.println(">> imu"); }
    else if (c == 'b') { stream_mode = BOTH; Serial.println(">> both"); }
    else if (c == 'x' || c == 'X') { stream_mode = STOP; Serial.println(">> stop"); }
    else if (c == 's') print_status();
  }

  if (stream_mode != STOP && millis() - last_ms >= 200) {
    last_ms = millis();
    if (stream_mode == ENC_ONLY || stream_mode == BOTH)
      Serial.printf("  enc1=%.3f rad  enc2=%.3f rad\n",
        encoder1.getAngle(), encoder2.getAngle());
    if (stream_mode == IMU_ONLY || stream_mode == BOTH) {
      if (imu.dataReady()) {
        imu.getAGMT();
        Serial.printf("  acc[mg] %.0f %.0f %.0f | gyr[dps] %.1f %.1f %.1f\n",
          imu.accX(), imu.accY(), imu.accZ(),
          imu.gyrX(), imu.gyrY(), imu.gyrZ());
      } else {
        Serial.println("  IMU no data");
      }
    }
  }
}
