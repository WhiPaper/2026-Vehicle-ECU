# 2026 Vehicle ECU

ESP32 기반 4WD skid-steer 차량 ECU 펌웨어다. 네 모터 제어, 좌·우
quadrature 엔코더, MPU6050, 폐루프 속도 제어와 micro-ROS 통신을 제공한다.
Raspberry Pi 5의 ROS 2 Jazzy 시스템이 상위 제어와 센서 융합을 담당한다.

> 기본 설정은 의도적으로 주행이 잠겨 있다. 실측한 엔코더·차체 기하·PID
> 값을 설정하기 전에는 모터 주행 명령을 거부한다.

## 대상 환경

- ESP32 DevKitC V4
- ESP-IDF 6.0.2
- FreeRTOS, ESP-IDF Component Manager
- Raspberry Pi 5, ROS 2 Jazzy, micro-ROS Agent

## 주요 기능

- 4채널 L9110S 모터 출력과 좌우 skid-steer 제어
- Registry PCNT 드라이버 기반 FL/FR/RL/RR quadrature 엔코더
- 유한-timeout ESP-IDF I2C 기반 MPU6050 측정과 정지 바이어스 보정
- 엔코더 PID 속도 제어, odometry, 명령·stall fail-safe
- Raspberry Pi 5와 USB-UART 기반 micro-ROS 통신

## 빠른 시작

권장 개발 환경은 저장소의 devcontainer다. 컨테이너를 연 뒤:

```bash
idf.py set-target esp32
idf.py build
```

로컬 설치, 설정, 플래시까지의 전체 절차는
[개발 시작 가이드](docs/getting-started.md)를 따른다.

## 문서 지도

- 처음 빌드·플래시: [개발 시작 가이드](docs/getting-started.md)
- 배선·전원·GPIO: [4WD 하드웨어 배선](docs/hardware-wiring.md)
- 모듈과 실행 흐름: [소프트웨어 구조](docs/software-architecture.md)
- 엔코더·기하·PID: [보정 가이드](docs/calibration.md)
- 단위·하드웨어·통합 검증: [시험 가이드](docs/testing.md)
- Pi에서 Agent 연결: [Raspberry Pi 5 micro-ROS 연결](docs/microros-rpi5.md)
- 토픽·QoS·fail-safe 계약: [micro-ROS ECU 설계](docs/microros-design.md)
- URDF·EKF·launch: [Raspberry Pi ROS 통합](docs/rpi-ros-integration.md)
- 외부 드라이버 선정 근거: [컴포넌트 선정 기준](docs/component-selection.md)

## 시험

기본 변경 검증:

```bash
python -m pip check
pre-commit run --all-files
idf.py build
```

모터·주행 단위 시험, 실제 모터 부하 시험과 통합 합격 조건은
[시험 가이드](docs/testing.md)를 참조한다. 실제 모터 시험은 차량을 고정하고
모든 바퀴를 띄운 뒤 수행한다.
