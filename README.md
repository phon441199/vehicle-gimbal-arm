# gimbalarm_ws — 차량 외란 환경에서의 능동 진동 절연 로봇팔

외란 관측기(ESO)와 태스크 공간 능동 서스펜션을 적용한 5축 로봇팔의 **2층 제어 구조**.
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
[2층]        태스크 공간 능동 서스펜션      (직접 구현)  — 어깨 + 팔꿈치
                │
[1층]        ESO 외란 관측기               (직접 구현)  — 손목 pitch
                │
[안쪽 루프]   AK40 임피던스 제어 (드라이버 내장, MIT 펌웨어)
             GM4108 FOC + PD   (직접 구현)
```

가장 안쪽 루프는 층수에 포함하지 않는다. AK40 쪽은 상용 드라이버에 내장된 기능이고, 손목
짐벌모터의 FOC+PD는 직접 구현했지만 둘 다 "그 위에 얹은 제어 층"이 아닌 구동단이기 때문이다.

- **1층 (ESO 외란 관측기)** — 손목 pitch 축에 적용. AK40에는 적용하지 못했는데, MIT 프로토콜이
  전류값을 회신하지 않아 구동 토크와 외란 토크를 분리할 수 없기 때문이다. 롤 축은 구현했으나
  발산이 확인되어 비활성화했다.
- **2층 (태스크 공간 능동 서스펜션)** — 컵 IMU 가속도를 대역통과(0.8–8 Hz)해 평면 내 팁 변위를
  추정하고, damped 2R 자코비안 의사역행렬로 어깨·팔꿈치 관절 보정량으로 변환해 MIT 홀드
  설정점 위에 더한다. 임피던스 홀드만으로는 기저 진동이 팁까지 거의 그대로 전달되므로
  (전달률 ≈ 1), 팁이 월드 좌표계에서 정지하도록 관절을 능동적으로 움직이는 방식이다.

구현은 [`src/teensy_firmware/arm_microros_teensy/`](src/teensy_firmware/arm_microros_teensy/arm_microros_teensy.ino)
한 파일에 들어 있다 (서스펜션 설계 근거는 `Active suspension` 주석 블록 참고).

## 저장소 구조

```
src/
├── teensy_firmware/       Teensy 4.1 최종 펌웨어 (2층 제어 + ESO + 서스펜션 + micro-ROS)
├── esp32_firmware/        이식 이전 초기 MCU 단계 (개발 기록용)
├── shaker_firmware/       외란 재현용 가진 장치(PlatformIO, ESP32)
├── gimbal_arm_controller/ ROS 2: 역기구학, 자세 제어, GUI 노드
├── gimbalarm_description/ ROS 2: URDF, RViz 설정
├── gimbal_bridge/         ROS 2: 브리지 노드
├── imu_test_visualizer/   ROS 2: IMU 데이터 시각화
├── micro_ros_msgs/        (미포함, 아래 "빌드" 참고)
├── micro-ROS-Agent/       (미포함, 아래 "빌드" 참고)
└── uno_imu_test/          IMU 단독 벤치 테스트 (버스 분리 진단용)
scripts/                   micro-ROS agent 실행 스크립트
```

`build/`, `install/`, `log/`는 colcon 빌드 산출물이라 git에 포함하지 않는다.

## 빌드

이 저장소는 `src/micro-ROS-Agent`, `src/micro_ros_msgs`를 포함하지 않는다. 둘 다 본인 작성 코드가
아니라 micro-ROS 프로젝트의 표준 패키지([micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent),
[micro_ros_msgs](https://github.com/micro-ROS/micro_ros_msgs))를 그대로 사용한 것이라 벤더링하지 않았다.
워크스페이스를 받은 뒤 아래처럼 따로 받아 넣으면 된다.

```bash
git clone https://github.com/phon441199/vehicle-gimbal-arm.git
cd vehicle-gimbal-arm/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
git clone https://github.com/micro-ROS/micro_ros_msgs.git
cd ..
colcon build
source install/setup.bash
ros2 launch micro_ros_agent micro_ros_agent.launch.py   # 또는 scripts/start_microros_agent.sh
```

Teensy/ESP32 펌웨어는 Arduino IDE(또는 arduino-cli)로 각 `.ino`를 빌드/업로드한다.
`shaker_firmware`는 PlatformIO 프로젝트 (`pio run -t upload`).

## 트러블슈팅 ↔ 코드 매핑

포트폴리오에 정리한 트러블슈팅 항목이 실제 어떤 코드에서 나온 것인지 링크한다.

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

- **언어·프레임워크**: C/C++ (Arduino/Teensy), ROS 2, micro-ROS
- **제어**: 임피던스 제어, PD 제어, 확장상태관측기(ESO) 기반 외란 보상, FOC(SimpleFOC), 자코비안 의사역행렬(damped least-squares)
- **통신·인터페이스**: CAN(MIT 프로토콜), SPI, I2C, PWM
- **센서**: 절대 엔코더(AS5048A), IMU(ICM-20948, MPU6050), 인라인 전류 센싱
- **기타**: 상보 필터, 대역통과 필터, 실시간 스케줄링, EEPROM 파라미터 영속화
