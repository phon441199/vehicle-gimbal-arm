# Teensy 4.1 펌웨어

메인 제어 MCU. AK40-10 QDD ×2(CAN, MIT 프로토콜),
GM4108 짐벌모터 ×2(SimpleFOC), IMU/엔코더(SPI, I2C), micro-ROS를 이 한 보드에서 구동한다.

| 폴더 | 내용 |
| --- | --- |
| [`arm_microros_teensy/`](arm_microros_teensy/arm_microros_teensy.ino) | **최종 통합 펌웨어.** 2층 제어(DOB 외란 관측기 + PD 제어 기반 가변 서스펜션), MIT 임피던스 홀드, micro-ROS 논블로킹 재접속, 열 보호, IMU 섹터 스냅 호밍, 중력 보상 구조까지 포함한 실제 동작 버전. |
| [`diagnostics/`](diagnostics/) | 하드웨어 결함을 분리하기 위해 작성한 진단 스케치. 포트폴리오 트러블슈팅 섹션과 1:1로 대응된다. |
| [`prototypes/`](prototypes/) | 최종본으로 통합되기 전 단계별로 검증한 개별 기능 스케치 (CAN 단독 구동, IMU-microROS 연동, 손목 레벨링 단독 테스트 등). |
| `PINMAP_teensy41.txt` | 실제 배선 기준 핀맵. |

## 제어 구조 (최종 펌웨어)

AK40의 임피던스 제어는 드라이버 펌웨어(MIT)에 내장된 기능이며, 이를 가장 안쪽 루프로 사용하고
그 위에 **두 개의 층을 직접 구현**했다. 안쪽 루프는 층수에 포함하지 않는다.

- **1층 — DOB 외란 관측기** (손목 pitch): 관측기 대역폭은 기계 공진(약 9 Hz) 아래인 6.78 Hz로
  튜닝했고, 컵 유무에 따라 관성 J를 바꿔 입력 게인 b₀ = Kt/J를 다시 계산한다. AK40에는 적용하지
  못했다. MIT 프로토콜이 전류값을 회신하지 않아 구동 토크와 외란 토크를 분리할 수 없기 때문이다.
  롤 축은 구현했으나 발산이 확인되어 비활성화했다.
- **2층 — PD 제어 기반 가변 서스펜션** (어깨+팔꿈치): 컵 IMU 가속도를 대역통과(0.8–8 Hz)해
  평면 내 팁 변위·속도를 추정하고, 변위 비례(P)·속도 비례(D, sky-hook 감쇠) 항으로 관절 보정량을
  만들어 MIT 홀드 설정점 위에 더한다. 두 게인은 런타임 조절식이고(0 = off), 서스펜션을 켤 때
  MIT kp를 120 → 30–40으로 낮춰 관절을 무르게 만든다. 보정량 배분에는 damped 2R 자코비안
  의사역행렬을 쓴다. 소스의 `Active suspension` 주석 블록에 설계 근거와 튜닝 이력이 있다.

## 트러블슈팅 ↔ 코드 매핑

| 증상 | 코드 |
| --- | --- |
| I2C 주소 변동 (AD0 플로팅) | [`diagnostics/i2c_scanner`](diagnostics/i2c_scanner/i2c_scanner.ino) |
| 열 보호 트립 / kp 램프다운 검증 | [`diagnostics/hw_selftest_teensy`](diagnostics/hw_selftest_teensy/hw_selftest_teensy.ino), [`diagnostics/hw_selftest`](diagnostics/hw_selftest/hw_selftest.ino) |
| 전원 재인가 시 관절각 복원 (IMU 섹터 판정) | [`arm_microros_teensy.ino`](arm_microros_teensy/arm_microros_teensy.ino) — `sector snap` 부팅 시퀀스 |
| 전류 센서 초기화 실패 (아날로그 채널 지정) | [`arm_microros_teensy.ino`](arm_microros_teensy/arm_microros_teensy.ino) — `WR_CS_A/B`, `WR2_CS_A/B` (A0–A3) |
| micro-ROS 도메인 ID / 논블로킹 재접속 | [`arm_microros_teensy.ino`](arm_microros_teensy/arm_microros_teensy.ino) |
| 손목 pitch 수평 유지 단독 검증 | [`prototypes/gimbal_level_hold`](prototypes/gimbal_level_hold/gimbal_level_hold.ino) |
| AK40 CAN(MIT 프로토콜) 단독 구동 검증 | [`prototypes/ak40_can_test`](prototypes/ak40_can_test/AK40_Impedance_Multi/AK40_Impedance_Multi.ino) |

## prototypes/ 에 대한 참고

`prototypes/gimbal_arm_full`은 개발 중반(2026-05)의 통합 시도로, AK40 CAN 구동과 손목 제어는
들어 있으나 micro-ROS·열 보호·섹터 호밍·서스펜션이 없다. `prototypes/gimbal_arm_firmware`는
그보다 더 이른 초기 버전이다. 둘 다 개발 과정 기록용으로만 남겨두었고, 실제 최종 동작 버전은
[`arm_microros_teensy/`](arm_microros_teensy/arm_microros_teensy.ino)다.

최종 펌웨어 헤더 주석 첫 줄이 `arm_microros_esp32.ino`로 남아 있으나 실제로는 Teensy 4.1
빌드다 (같은 주석 블록의 `PLATFORM:` 항목 참고).
