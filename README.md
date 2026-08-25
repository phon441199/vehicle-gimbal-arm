# gimbalarm_ws — 차량 외란 환경에서의 능동 진동 절연 로봇팔

외란 관측기(DOB)와 PD 제어 기반 가변 서스펜션을 적용한 5축 로봇팔의 **2층 제어 구조**.
AK40-10 QDD(어깨/팔꿈치, CAN·MIT 프로토콜)와 GM4108 짐벌모터(손목 pitch/roll, SimpleFOC)를
Teensy 4.1 한 대에서 구동하고, micro-ROS로 ROS 2(RViz2, Tkinter GUI)와 연결한다.

프로젝트 배경, 목표, 결과 정리는 포트폴리오 문서를 참고. 이 저장소는 그 문서에서 언급한
트러블슈팅과 구현이 실제로 어떤 코드였는지를 보여주기 위한 소스다.

담당 범위: 부품 선정·구매, 기구 설계·제작, 회로 구성, 펌웨어 및 제어 전반 (팀 프로젝트 3인 중).

## 제어 구조

AK40의 임피던스 제어는 드라이버 펌웨어(MIT)에 내장된 기능이다. 이 프로젝트에서는 그것을
**가장 안쪽 루프로 그대로 사용**하고, 그 위에 **두 개의 제어 층을 직접 구현**했다.

```
[상위]       micro-ROS / ROS 2  ──  RViz2, Tkinter GUI
                │
[2층]        PD 제어 기반 가변 서스펜션     (직접 구현)  — 어깨 + 팔꿈치
                │
[1층]        DOB 외란 관측기               (직접 구현)  — 손목 pitch
                │
[안쪽 루프]   AK40 임피던스 제어 (드라이버 내장, MIT 펌웨어)
             GM4108 FOC + PD   (직접 구현)
```

- **1층 (DOB 외란 관측기)** — 손목 pitch 축에 적용. 관측기 대역폭은 기계 공진(약 9 Hz에서 발생) 아래인
  6.78 Hz로 튜닝했고(일반적인 크기의 원통형 물컵속 물의 고유진동수는 2~4Hz 부근으로 근사된다), 컵 유무에 따라 관성 J를 바꿔 입력 게인 b₀ = Kt/J를 다시 계산한다.
- **2층 (PD 제어 기반 가변 서스펜션)** — AK40 의 MIT프로토콜을 통해 필요한 위치, 속도, Kp, Kd를 모터로 전송한다. 모터는 피드백루프로 기계적 스프링 - 댐퍼시스템과 유사하게 운동하여  물컵까지 전달되는 저주파 외란을 소산시키는 역할을 한다.

구현은 [`src/teensy_firmware/arm_microros_teensy/`](src/teensy_firmware/arm_microros_teensy/arm_microros_teensy.ino)
한 파일에 들어 있다 (서스펜션 설계 근거는 `Active suspension` 주석 블록 참고).

## 저장소 구조

```
src/
├── teensy_firmware/       Teensy 4.1 최종 펌웨어 (2층 제어 + DOB + 서스펜션 + micro-ROS)
├── esp32_firmware/        중간중간 디버깅용으로 사용한 ESP32 스케치
├── shaker_firmware/       외란 재현용 가진 장치(PlatformIO, ESP32)
├── gimbal_arm_controller/ ROS 2: 역기구학, 자세 제어, GUI 노드
├── gimbalarm_description/ ROS 2: URDF, RViz 설정
├── gimbal_bridge/         ROS 2: 브리지 노드
├── imu_test_visualizer/   ROS 2: IMU 데이터 시각화
└── uno_imu_test/          IMU 단독 벤치 테스트 (버스 분리 진단용)
scripts/                   micro-ROS agent 실행 스크립트
```

`build/`, `install/`, `log/`는 colcon 빌드 산출물이라 git에 포함하지 않는다.


## 트러블슈팅 ↔ 코드 매핑

노션에 정리한 트러블슈팅 항목과 연결되는 코드들이다.

| 항목 | 코드 |
| --- | --- |
| micro-ROS 노드 미검출 — 도메인 ID 불일치 | [`arm_microros_teensy.ino`](src/teensy_firmware/arm_microros_teensy/arm_microros_teensy.ino) — 논블로킹 ping/재접속 상태 기계 |
| 전원 재인가 시 관절각 불확정 — IMU 기반 섹터 판정 | [`arm_microros_teensy.ino`](src/teensy_firmware/arm_microros_teensy/arm_microros_teensy.ino) — `sector snap` 부팅 시퀀스 |
| I2C 주소 변동 — AD0 핀 플로팅 | [`diagnostics/i2c_scanner`](src/teensy_firmware/diagnostics/i2c_scanner/i2c_scanner.ino) |
| 전류 센서 초기화 실패 — 아날로그 채널 지정 오류 | [`arm_microros_teensy.ino`](src/teensy_firmware/arm_microros_teensy/arm_microros_teensy.ino) — `WR_CS_A/B`, `WR2_CS_A/B` (A0–A3) |
| 엔코더 SPI 미동작 — 케이블 색상 규약 불일치 | [`src/esp32_firmware/enc_imu_spi_test`](src/esp32_firmware/enc_imu_spi_test/) |
| CAN 통신 두절 — 트랜시버 동작 전압 | [`prototypes/ak40_can_test`](src/teensy_firmware/prototypes/ak40_can_test/AK40_Impedance_Multi/AK40_Impedance_Multi.ino) |
| SPI 초기화 간헐 실패 — 물리적 접촉 불량 | [`esp32_firmware/spi_miso_probe`](src/esp32_firmware/spi_miso_probe/), [`uno_imu_test`](src/uno_imu_test/uno_imu_test.ino) (IMU를 별도 보드로 분리해 원인 격리) |

세부 설명은 각 하위 폴더의 README 참고: [`teensy_firmware/README.md`](src/teensy_firmware/README.md),
[`esp32_firmware/README.md`](src/esp32_firmware/README.md).

## 기술 스택

- **언어·프레임워크**: C/C++ (Arduino IDE), Python (ROS 2 포함)
- **제어**: 임피던스 제어, PD 제어, 외란관측기(DOB), FOC (SimpleFOC 프레임워크 사용)
- **통신·인터페이스**: CAN (MIT 프로토콜), SPI, I2C, PWM
- **센서**: 절대 엔코더 (AS5048A), IMU (ICM-20948, MPU6050), 전류 센서 (INA181A2)
- **기타**: Analytic Inverse Kinematics, Quintic trajectory, BandPass 필터, 상보 필터
