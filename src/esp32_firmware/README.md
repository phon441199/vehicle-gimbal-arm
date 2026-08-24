# ESP32 (LOLIN D32) 펌웨어 — 이식 이전 단계

프로젝트 초기 MCU. 이후 CAN 컨트롤러 부재, 처리 성능/핀 여유 문제로 **Teensy 4.1로 이식**했다
(→ [`../teensy_firmware/`](../teensy_firmware/)). 이 폴더는 그 이전까지 검증했던 개별 기능 스케치를
개발 기록으로 남긴 것이며, 이후 기능 추가는 Teensy 쪽에서만 이루어졌다.

| 스케치 | 내용 |
| --- | --- |
| `arm_microros_esp32` | micro-ROS 연동 초기 버전 |
| `gimbal_level_hold_esp32` | 손목 수평 유지 초기 실험 |
| `imu_microros_esp32` | IMU → micro-ROS 퍼블리시 테스트 |
| `motor_test_esp32` | 모터 단독 구동 테스트 |
| `hw_selftest_esp32` | 하드웨어 자가진단 |
| `enc_imu_spi_test`, `spi_miso_probe` | SPI 버스 공유 시 엔코더/IMU 간헐 초기화 실패를 분리하기 위한 진단 스케치 |

`spi_miso_probe`는 포트폴리오의 "SPI 초기화 간헐 실패 — 물리적 접촉 불량" 트러블슈팅 항목에서
버스 신호 무결성을 먼저 의심하고 배제하는 데 사용한 스케치다.
