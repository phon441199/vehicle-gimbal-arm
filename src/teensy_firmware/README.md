# Teensy 4.1 펌웨어

ESP32(LOLIN D32)에서 이식한 최종 MCU. AK40-10 QDD ×2(CAN, MIT 프로토콜),
GM4108 짐벌모터 ×2(SimpleFOC), IMU/엔코더(SPI, I2C), micro-ROS를 이 한 보드에서 구동한다.

| 폴더 | 내용 |
| --- | --- |
| [`gimbal_arm_full/`](gimbal_arm_full/gimbal_arm_full.ino) | **최종 통합 펌웨어.** 5축 구동, ESO 외란 관측기, 임피던스 홀드, 열 보호, IMU 기반 관절각 복원까지 포함된 실제 동작 버전. |
| [`diagnostics/`](diagnostics/) | 하드웨어 결함을 분리하기 위해 작성한 진단 스케치. 포트폴리오 트러블슈팅 섹션과 1:1로 대응된다. |
| [`prototypes/`](prototypes/) | `gimbal_arm_full`로 통합되기 전 단계별로 검증한 개별 기능 스케치 (CAN 단독 구동, IMU-microROS 연동, 손목 레벨링 단독 테스트 등). |
| `PINMAP_teensy41.txt` | 실제 배선 기준 핀맵. |

## 트러블슈팅 ↔ 코드 매핑

| 증상 | 코드 |
| --- | --- |
| I2C 주소 변동 (AD0 플로팅) | [`diagnostics/i2c_scanner`](diagnostics/i2c_scanner/i2c_scanner.ino) |
| 열 보호 트립 / kp 램프다운 검증 | [`diagnostics/hw_selftest_teensy`](diagnostics/hw_selftest_teensy/hw_selftest_teensy.ino), [`diagnostics/hw_selftest`](diagnostics/hw_selftest/hw_selftest.ino) |
| 전원 재인가 시 관절각 복원 (IMU 섹터 판정) | [`gimbal_arm_full/gimbal_arm_full.ino`](gimbal_arm_full/gimbal_arm_full.ino) 부팅 시퀀스 |
| 손목 pitch 수평 유지 단독 검증 | [`prototypes/gimbal_level_hold`](prototypes/gimbal_level_hold/gimbal_level_hold.ino) |
| micro-ROS 도메인 ID / 논블로킹 재접속 | [`prototypes/arm_microros_teensy`](prototypes/arm_microros_teensy/arm_microros_teensy.ino) |
| AK40 CAN(MIT 프로토콜) 단독 구동 검증 | [`prototypes/ak40_can_test`](prototypes/ak40_can_test/AK40_Impedance_Multi/AK40_Impedance_Multi.ino) |

`prototypes/gimbal_arm_firmware`는 `gimbal_arm_full` 이전의 초기 통합 버전(관절 3개 미만, ESO 미적용)으로,
개발 과정 기록용으로만 남겨두었다.
