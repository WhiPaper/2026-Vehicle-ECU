# 시험 가이드

시험은 순수 계산 로직, ESP32/QEMU 단위 시험, 실제 모터 하드웨어 시험으로
나뉜다. 실차를 움직이는 시험은 반드시 차체를 고정하고 바퀴를 지면에서
띄운 상태에서 시작한다.

## CI와 로컬 기본 검증

CI는 고정된 devcontainer에서 다음 항목을 확인한다.

- Python 의존성 무결성과 pre-commit 검사
- 루트 펌웨어의 clean build
- `motor`, `drive` 단위 시험 애플리케이션 build
- 모터 하드웨어 시험 애플리케이션 build
- Component Manager lock file이 빌드 후 바뀌지 않는지 확인

로컬 변경의 최소 검증:

```bash
python -m pip check
pre-commit run --all-files
idf.py build
```

## 모터 단위 시험

`motor` 컴포넌트의 skid-steer mixer를 실제 GPIO 구동 없이 검증한다.

```bash
cd components/motor/test_apps/unit
idf.py set-target esp32
idf.py build
idf.py qemu
```

Unity prompt에서 `*`를 입력해 모든 case를 실행한다. 실제 ESP32에서도
실행할 수 있다.

```bash
idf.py -p /dev/serial/by-id/<CP2102-device> flash monitor
```

퍼센트 API(`motor_set_percent()`, `motor_set_tank_percent()`,
`motor_drive_percent()`)의 범위는 `-100..100`이고, `*_normalized()` API의
범위는 `-1000..1000`이다.

## 주행 계산 단위 시험

센서 환산, 엔코더 RPM, 주행 기하와 PID 수학을 검증한다.

```bash
cd components/drive/test_apps/unit
idf.py set-target esp32
idf.py build
idf.py -p /dev/serial/by-id/<CP2102-device> flash monitor
```

Unity prompt에서 `*`를 입력해 모든 case를 실행한다.

## 모터 하드웨어 시험

> 경고: 이 애플리케이션은 실제 모터를 구동한다. 차량을 고정하고 바퀴를
> 띄우며, 비상 정지 수단을 손이 닿는 곳에 둔다.

```bash
cd components/motor/test_apps/hardware
idf.py set-target esp32
idf.py -p /dev/serial/by-id/<CP2102-device> flash monitor
```

Enter를 눌러 case 목록을 표시한 뒤 필요한 시험 번호만 입력한다. 차량을
완전히 확보하지 않았다면 `*`로 전체 시험을 실행하지 않는다.

시험 애플리케이션은 다음 동작을 제공한다.

- 바퀴별 회전, 전체 전진·후진과 pivot 명령(60%)
- 전륜 또는 후륜 25%, 50%, 75%, 100% 12초 부하 시험
- 전체 4륜 12초 연속 전진과 3초 완전 출력 시험
- 전·후륜별 5% 단위 최소 기동 출력과 최소 유지 출력 측정

최소 기동 시험은 5%부터 증가시키며 두 바퀴가 모두 출발하는 값을 기록한다.
최소 유지 시험은 100%에서 감소시키며 두 바퀴가 계속 회전하는 마지막 값을
기록한다. 이 애플리케이션은 측정 왜곡을 막기 위해 startup boost를 끈다.

부하 시험 중에는 매초 진행 로그와 함께 다음을 관찰한다.

- L9110S와 모터 온도
- 모터 전원 전압 강하
- ESP32 reset 또는 micro-ROS 통신 끊김
- 좌우 회전 방향과 비정상 소음

정상 PWM 주파수는 20 kHz다. 파형이나 기동 문제를 분석할 때만
`idf.py menuconfig`의 **Board → Motor PWM frequency**를 1000 Hz로
변경하고, 시험 후 반드시 20000 Hz로 복원한다.

## 실차 통합 합격 조건

[보정 절차](calibration.md)를 완료한 뒤 다음 조건을 확인한다.

- `/imu/data_raw`는 기본 50 Hz, `/odom`과 `/joint_states`는 기본 30 Hz
- `/diagnostics`는 정상 상태와 보정 여부를 올바르게 보고
- `/cmd_vel` 중단 후 500 ms 안에 모터 정지
- 엔코더 stall 시 500 ms 후 출력 정지와 drive `ERROR`
- Agent 재시작 후 과거 명령을 재생하지 않고 자동 재연결

ROS 측 명령과 확인 절차는
[Raspberry Pi ROS 통합 가이드](rpi-ros-integration.md#확인-명령)를 따른다.
