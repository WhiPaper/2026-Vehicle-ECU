# Raspberry Pi 5 micro-ROS 연결

ESP32 DevKitC의 CP2102 USB 케이블 하나로 ESP32 전원과 UART 통신을 함께
제공한다. 펌웨어는 UART0을 921600 baud XRCE-DDS transport로 전용 사용하며
일반 콘솔 로그는 출력하지 않는다. Raspberry Pi 5에는 ROS 2 Jazzy와
Jazzy용 `micro_ros_agent`가 필요하다.

이 문서는 장치 연결과 Agent 운용 절차만 다룬다. ROS wire 계약과 fail-safe
설계는 [micro-ROS ECU 설계](microros-design.md), URDF·EKF·launch 개발은
[Raspberry Pi ROS 통합 가이드](rpi-ros-integration.md)를 참조한다.

## Agent 실행

장치 이름이 재부팅 후에도 안정적인 `/dev/serial/by-id` 경로를 권장한다.

```bash
ls -l /dev/serial/by-id/
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/serial/by-id/<CP2102-device> -b 921600 -v6
```

설치된 Agent 버전이 `-b`를 받지 않으면 `--baudrate 921600`을 사용한다.
권한 오류가 나면 로그인 사용자를 `dialout` 그룹에 추가한 뒤 다시
로그인한다.

```bash
sudo usermod -aG dialout "$USER"
```

ESP32를 플래시할 때는 Agent를 종료해 serial port를 해제한다.

```bash
idf.py -p /dev/serial/by-id/<CP2102-device> flash
```

Agent를 다시 실행하면 펌웨어가 자동으로 연결을 탐지하고 ROS 시각을
동기화한다. 케이블 분리나 Agent 재시작 후에도 500 ms 간격으로 재접속한다.

## 연결 확인

현재 펌웨어의 토픽 이름, 타입과 기본 주기는
[ROS 인터페이스 계약](microros-design.md#ros-인터페이스-계약)에 정리되어
있다. `diagnostics`는 표준 `diagnostic_msgs/msg/DiagnosticArray`다.

간단한 연결 시험:

```bash
ros2 topic list
ros2 topic hz /imu/data_raw
ros2 topic echo /diagnostics
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'
```

마지막 명령 후 500 ms가 지나면 자동 정지한다. 기하·엔코더·PID 값이
보정되지 않았다면 `/cmd_vel`은 안전하게 거부된다.

## 문제 해결

- Topic이 없으면 Agent의 device와 baud rate, USB 케이블의 data 지원 여부를
  확인한다.
- `Device or resource busy`이면 실행 중인 Agent나 serial monitor를
  종료한다.
- 반복 재접속이면 921600 baud 양쪽 설정과 USB 전원 안정성을 확인한다.
- 모터 구동 시 통신이 끊기면 모터 전원과 ESP32 전원을 분리하고 공통 GND,
  bulk capacitor, 모터 노이즈 억제를 점검한다.
- diagnostics의 `vehicle_ecu/drive.calibrated=false`는 통신 장애가 아니라
  의도된 미보정 잠금이다. 먼저
  [보정 절차](calibration.md)를 완료한다.

Agent를 `-v6`로 실행했을 때 ESP32 client의 ping 또는 session 요청이 전혀
보이지 않으면 entity/QoS보다 앞선 펌웨어 실행 또는 UART transport 문제다.
session 요청은 보이지만 토픽이 없으면 diagnostics의 `last_entity_stage`와
`last_rcl_error`를 확인한다. IMU만 멈추면 `vehicle_ecu/imu.state`,
`last_error`, `data_age_ms`로 I2C와 보정 상태를 확인한다.

현재 빌드와 실제 flash app을 비교하려면 Agent를 종료한 뒤 app 크기만큼
읽어 SHA-256 또는 byte 비교를 수행한다.

```bash
app_size="$(stat -c %s build/vehicle_ecu.bin)"
python -m esptool --chip esp32 -p /dev/serial/by-id/<CP2102-device> \
  read-flash 0x10000 "$app_size" flashed-app.bin
sha256sum build/vehicle_ecu.bin flashed-app.bin
cmp build/vehicle_ecu.bin flashed-app.bin
```

micro-ROS ESP-IDF Jazzy 컴포넌트는
`components/ros_bridge/idf_component.yml`의 Git
dependency로 관리하며 commit
`b84f9ddda4213c0aa71e08838a384a6adc0f4ddf`에 고정되어 있다. 실제 소스는
Component Manager가 `managed_components/`에 내려받는다.
