# ESP32 (LOLIN D32) 펌웨어 — 디버깅용

메인 제어는 Teensy 4.1([`../teensy_firmware/`](../teensy_firmware/))에서 동작한다.
이 폴더는 프로젝트 중반에 문제를 좁히기 위해 별도 보드로 분리해 돌린 스케치들이다.

| 스케치 | 내용 |
| --- | --- |
| `enc_imu_spi_test`, `spi_miso_probe` | SPI 버스 공유 시 엔코더/IMU 간헐 초기화 실패 원인 분리 |
| `hw_selftest_esp32` | 하드웨어 자가진단 |
| `motor_test_esp32` | 모터 단독 구동 확인 |
| `imu_microros_esp32` | IMU → micro-ROS 퍼블리시 확인 |
| `gimbal_level_hold_esp32` | 손목 수평 유지 단독 확인 |
| `arm_microros_esp32` | micro-ROS 연동 확인 |
