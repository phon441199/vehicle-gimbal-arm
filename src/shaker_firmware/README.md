# 이중 서보 조화가진장치 — IMU 기반 위상동기 제어

ESP32 + 연속회전 서보 2개(DS3345) + 크랭크-슬라이더로 30×30 cm 판을 상하 가진(≈ 2 Hz)하고,
IMU(MPU-6500) 1개로 두 서보의 위상 드리프트를 보정해 판 수평을 유지하는 펌웨어.

## 빌드 / 업로드 — 환경별 (셋 중 편한 것)

### A. PlatformIO + VS Code (가장 쉬움)
1. VS Code + PlatformIO 확장 설치
2. 이 폴더(`platformio.ini` 있는 곳) 열기
3. 보드 연결 후 PlatformIO: **Upload**
4. 시리얼 모니터 **115200** baud
> 라이브러리(`madhephaestus/ESP32Servo`)는 자동 설치됨.

### B. PlatformIO Core (CLI, VS Code 불필요)
1. PlatformIO Core 설치: `pip install platformio` (또는 공식 설치 프로그램)
2. 이 폴더에서:
   ```
   pio run -t upload                 # 빌드 + 업로드
   pio device monitor -b 115200      # 시리얼 모니터
   ```
> 기존 폴더 구조 그대로 동작. 라이브러리 자동 설치.

### C. Arduino IDE (PlatformIO 불필요)
1. **보드 매니저**: "esp32" (Espressif Systems) 설치
2. **라이브러리 매니저**: "ESP32Servo" (madhephaestus) 설치
3. `arduino/servo_exciter/servo_exciter.ino` 열기 (코드는 src/main.cpp와 동일)
4. 보드 = "ESP32 Dev Module" (또는 DOIT ESP32 DEVKIT V1), 포트 선택 후 **업로드**
5. 시리얼 모니터 **115200** baud
> Arduino는 폴더명=스케치명 규칙이라 반드시 `servo_exciter` 폴더째로 열 것.

### D. 그냥 코드만 보기
`src/main.cpp`(또는 `arduino/.../servo_exciter.ino`)는 텍스트라 메모장·웹브라우저로 열림. 빌드하려면 A·B·C 중 하나 필요.

## 배선
| 부품 | ESP32 핀 |
|---|---|
| 서보 1 (Master) 신호 | GPIO 18 |
| 서보 2 (Slave) 신호 | GPIO 19 |
| IMU SDA | GPIO 21 |
| IMU SCL | GPIO 22 |
| IMU VCC / GND | 3.3V / GND (공통) |
| 서보 전원 | 외부 5~6V, GND 공통 |

## 시리얼 명령
| 키 | 동작 |
|---|---|
| `c` / `a` | 정방향 / 역방향 운전 시작 |
| `s` | 정지 |
| `y` | 동기보정 ON/OFF |
| `d` | 디버그 출력(err, trim 등) ON/OFF |
| `k` | 현재 게인/상태 출력 |
| `2`/`1` | Kp +/- |
| `4`/`3` | Ki +/- |
| `f` | 보정 부호 반전 (발산 시) |
| `m` | I2C 버스 스캔 |
| `i` | IMU 원시값 진단 |

## 주요 파라미터 (src/main.cpp 상단)
- `STOP_PULSE = 1513` µs — DS3345 정지값(데드밴드 1504~1522 중앙)
- `BASE_OFFSET = 400` — 기준 구동속도(가진 ≈ 2 Hz)
- `Kp = 8`, `Ki = 40` — 위상동기 PI 게인(적분 위주)
- `VERT_ACC_AXIS = 2`(accel Z), `ROLL_GYR_AXIS = 1`(gyro Y) — IMU 축(실측 확정)

## 알아둘 점
- IMU를 재연결한 뒤엔 **ESP32 리셋 1회** 권장(부팅 시 I2C 버스복구 수행).
- 게인은 전원/리셋 시 기본값으로 복귀. 시리얼로 바꾼 값은 임시.
- 잔류 미세 진동(~15-20 dps)은 기계적 한계이며 위상보정 대상이 아님.

## 파일 구성
- `src/main.cpp` — 펌웨어 본체 (PlatformIO)
- `platformio.ini` — 보드/라이브러리 설정 (PlatformIO)
- `arduino/servo_exciter/servo_exciter.ino` — Arduino IDE용 (내용 동일)
- `control_structure.docx` — 제어구조 설명 문서
- `block_diagram4.png` — 제어 블록다이어그램 (최종본)
- `imu_freq_clean.png` — IMU 기반 가진 주파수 그래프 (최종본)
