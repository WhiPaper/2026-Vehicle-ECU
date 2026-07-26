# micro-ROS ECU 설계

이 문서는 ESP32 ECU와 Raspberry Pi 5 사이의 micro-ROS 계약에 대한 기준
문서다. 연결 및
플래시 절차는 [Raspberry Pi 5 micro-ROS 연결](microros-rpi5.md), Pi의 ROS
패키지 구성은 [Raspberry Pi ROS 통합 가이드](rpi-ros-integration.md)를
따른다.

## 시스템 경계

```text
ESP32 / FreeRTOS                         Raspberry Pi 5 / ROS 2 Jazzy

motor, encoder, MPU6050
        |
  drive / imu
        |
  message mapper
        |
  ROS entities
        |
  session manager
        |
 UART transport  ===== XRCE-DDS =====  micro_ros_agent
                                               |
                              robot_state_publisher / EKF
                                               |
                                  teleop, navigation, logging
```

ESP32는 센서 취득, 50 Hz 휠 속도 PID, skid-steer 변환, 휠 odometry 적분과
모터 fail-safe를 소유한다. Pi는 URDF, 센서 융합, 동적 TF와 상위 애플리케이션을
소유한다. ESP32가 보내는 `odom`은 휠 기반 raw odometry이며 최종 추정값은
Pi의 `robot_localization`이 발행한다.

### ECU 계층

| 계층 | 책임 | 소유하지 않는 것 |
|---|---|---|
| UART transport | UART0 설정, XRCE-DDS byte read/write, port open/close | ROS 엔티티와 재접속 정책 |
| Session manager | Agent 탐지, 연결 상태 전이, 엔티티 생성·해제, 시각 동기화 | 센서 및 제어 계산 |
| ROS entities | 노드, publisher/subscription, executor와 QoS | 하드웨어 접근 |
| Message mapper (`ros_messages`) | `drive_state_t`, `imu_sample_t`와 ROS 메시지 간 변환 및 메시지 저장공간 관리 | 상태의 원본 소유권 |
| Diagnostics mapper (`ros_diagnostics`) | fault snapshot, 보정·통신 상태를 `DiagnosticArray`로 변환 | 세션 전이와 하드웨어 복구 |
| `drive` | 명령 watchdog, PID, stall 검출, 휠 odometry snapshot | ROS 연결 |
| `imu` | MPU6050 취득, SI 단위 변환, 정지 bias 보정과 복구 | 센서 융합 |

`drive` 태스크가 제어 상태의 단일 writer이고, ROS 계층은 공개 snapshot API로
읽는다. `cmd_vel` callback은 검증된 명령만 `drive`에 넘긴다. 연결 상태나
진단 발행이 지연되어도 제어 태스크가 기다리지 않아야 한다.

## 실행 및 연결 상태

주행 제어는 주기 20 ms(50 Hz), 우선순위 8을 기준으로 한다. ROS 태스크는
더 낮은 우선순위에서 executor와 발행 작업을 수행한다.

| 상태 | 동작 | 전이 |
|---|---|---|
| `WAITING_AGENT` | 모터 정지, 500 ms 간격으로 Agent ping | 응답 시 `CREATING_ENTITIES` |
| `CREATING_ENTITIES` | 메시지와 ROS 엔티티 생성, epoch 동기화 | 성공 시 `CONNECTED`, 실패 시 `RECOVERING` |
| `CONNECTED` | 명령 처리와 주기 발행, 연결 health 확인 | Agent 손실 또는 치명적 RCL 오류 시 `RECOVERING` |
| `RECOVERING` | 즉시 `drive_stop()`, 엔티티·메시지·transport를 생성의 역순으로 해제 | 정리 후 `WAITING_AGENT` |

Agent 단절 판정과 엔티티 정리는 명령 watchdog과 별개다. 어느 쪽이 먼저
작동하든 모터 정지는 idempotent해야 한다. 엔티티 destroy timeout은
연결이 끊긴 경우 0으로 설정해 복구가 네트워크 응답을 기다리지 않게 한다.

연결마다 `rmw_uros_sync_session()`으로 epoch을 동기화하고 실패하면 1초
간격으로 다시 시도한다. 동기화 전에는 sensor 및 odometry 발행을 보류하고
stamp 0의 diagnostics로 `time_synchronized=false`를 알린다. Pi의 융합
노드는 동기화되지 않았거나 미래·과거 허용 범위를 벗어난 표본을 사용하지
않는다.

## ROS 인터페이스 계약

노드 이름은 `vehicle_ecu`다. 코드에는 선행 `/` 없는 상대 토픽 이름을
사용한다.

| 방향 | 이름 | 타입 | QoS | 목표 주기 |
|---|---|---|---|---:|
| 구독 | `cmd_vel` | `geometry_msgs/msg/Twist` | reliable, keep last 1 | 명령 발생 시 |
| 발행 | `imu/data_raw` | `sensor_msgs/msg/Imu` | best effort, keep last 1 | 50 Hz |
| 발행 | `joint_states` | `sensor_msgs/msg/JointState` | best effort, keep last 1 | 30 Hz |
| 발행 | `odom` | `nav_msgs/msg/Odometry` | best effort, keep last 1 | 30 Hz |
| 발행 | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable, keep last 1 | 5 Hz |

`cmd_vel`에서는 `linear.x`(m/s)와 `angular.z`(rad/s)만 사용한다. 둘 중 하나가
NaN 또는 infinity면 전체 명령을 거절하고 정지한다. 나머지 축은 무시한다.
마지막 유효한 비영점 명령을 받은 뒤 500 ms 안에 새 명령이 없으면 정지한다.
0 명령은 즉시 정지시키며 watchdog을 다시 arm하지 않는다. 기하, CPR 또는
최대 RPM이 보정되지 않은 ECU는 비영점 명령을 거절한다.

### frame과 필드

- `imu/data_raw.header.frame_id`는 `imu_link`다. orientation은 추정하지
  않으며 `orientation_covariance[0] = -1`로 표시한다.
- `joint_states.header.frame_id`는 `base_link`이며 이름은
  `left_wheel_joint`, `right_wheel_joint`다. 같은 쪽 앞·뒤 바퀴는 한 엔코더
  값으로 대표된다는 하드웨어 한계를 소비자가 알아야 한다.
- `odom.header.frame_id`는 `odom`, `child_frame_id`는 `base_link`다. pose와
  twist는 휠 엔코더만으로 계산한다.
- ESP32는 `/tf` 또는 `/tf_static`을 발행하지 않는다. `odom → base_link`는
  Pi의 EKF만 발행하고 `base_link → imu_link` 및 차체 고정 변환은 URDF가
  제공한다.

covariance의 0은 “완벽한 측정”으로 해석될 수 있으므로 알 수 없는 값을
0으로 두지 않는다. 휠 odometry covariance는 직진·회전 반복 시험의 분산,
IMU 각속도·가속도 covariance는 정지 및 일정 운동 표본의 분산으로 산정한다.
보정 전에는 보수적인 큰 양의 분산을 사용한다. 측정하지 않는 odometry의
z, roll, pitch 항목은 Pi의 `two_d_mode`로 제한하며 융합 입력으로 선택하지
않는다.

### 표준 diagnostics 계약

`diagnostics.status`에는 최소한 다음 세 status를 둔다.

| status name | level 의미 | 필수 key |
|---|---|---|
| `vehicle_ecu/transport` | 연결 또는 시각 문제 | `session_state`, `agent_connected`, `time_synchronized`, `last_error` |
| `vehicle_ecu/drive` | 보정·명령·엔코더·stall·모터 상태 | `calibrated`, `command_active`, `command_age_ms`, `encoder_ok`, `stalled`, `motor_ok`, `fault_mask` |
| `vehicle_ecu/imu` | IMU 초기화·읽기·발행 상태 | `imu_ok`, `calibrated`, `last_error` |

정상은 `OK`, 기능은 유지되지만 시각 미동기 또는 IMU 부재이면 `WARN`,
주행 불가나 motor/encoder/stall 오류이면 `ERROR`다. 문자열은 고정된
사전 할당 buffer 또는 초기화 시 할당한 저장소를 재사용한다.

이전 `std_msgs/msg/UInt32` 진단에서 표준 계약으로 옮긴 대응은 다음과 같다.

| 기존 bit | 의미 | 목표 status/key |
|---:|---|---|
| 0 | 주행 미보정 | `drive.calibrated=false`, `ERROR` |
| 1 | 명령 timeout | `drive.command_active=false`, `WARN` |
| 2 | 엔코더 오류 | `drive.encoder_ok=false`, `ERROR` |
| 3 | stall | `drive.stalled=true`, `ERROR` |
| 4 | 모터 오류 | `drive.motor_ok=false`, `ERROR` |
| 16 | IMU 오류 | `imu.imu_ok=false`, `WARN` |
| 17 | epoch 미동기 | `transport.time_synchronized=false`, `WARN` |

메시지 타입 변경은 wire 호환이 아니므로 이전 펌웨어와 현재 Pi 패키지를
혼용하지 않는다.

## 자원과 대역폭

메시지 sequence와 string, executor, node 및 엔티티는
`CREATING_ENTITIES`에서 한 번 초기화한다. `CONNECTED`의 발행 루프에서는
할당·해제를 하지 않고 같은 메시지 storage를 갱신해 재사용한다. 부분 생성
실패도 생성 완료 mask에 따라 안전하게 정리한다.

`rmw_microxrcedds`의 목표 정적 한도는 node 1, publisher 4, subscription 1,
service 0, client 0, history 2다. publisher나 subscriber를 추가할 때는
`app-colcon.meta`, RAM 사용량과 직렬 부하를 같은 변경에서 갱신한다.

921600 baud, 8-N-1의 wire 상한은 약 92,160 byte/s다. `Imu`와 `Odometry`는
각각 covariance 고정 배열 3개와 2개를 항상 직렬화하므로 wire 크기가 크다.
정렬과 frame 문자열을 포함한 1차 상한 추정은 IMU 약 320 B, JointState
약 120 B, Odometry 약 700 B, DiagnosticArray 약 500 B다. 초기 기본 주기를
적용하면 application payload는 약 43.1 kB/s로 wire 상한의 약 47%다.
XRCE framing, reliable ACK와 재전송을 포함한 실제 부하는 계측으로 확인한다.

초기 구현 기본값은 이 위험을 줄이기 위해 IMU 50 Hz, odometry와 JointState
30 Hz, diagnostics 5 Hz를 사용한다. 구현 수용 시 실제 직렬화 크기, 10분
평균·peak byte rate, 재전송과 publish 실패를 측정하고 지속 사용률 60% 이하,
peak 80% 이하를 통과해야 한다. 통과하지 못하면 JointState 20 Hz 순으로
발행률을 더 낮추고 diagnostics 문자열 길이를 제한한다.
샘플링과 50 Hz 주행 제어 주기는 발행률과 독립적으로 유지한다. 그래도
부족하면 baud rate 또는 transport 변경을 별도 설계 변경으로 다루며,
안전 의미가 있는 `cmd_vel`과 diagnostics의 reliable QoS는 낮추지 않는다.

## Fail-safe와 수용 조건

다음 조건은 모두 모터 정지를 일으켜야 한다.

- Agent 또는 USB 연결 손실
- 유효한 `cmd_vel`의 500 ms timeout
- NaN/infinity 명령
- 차량 기하, CPR 또는 최대 RPM 미보정
- 엔코더 읽기 오류 또는 목표 속도 중 stall
- 모터 출력 오류

Agent 미실행 부팅, 엔티티 생성 실패, Agent 종료, USB 분리·복원, ECU
재부팅, epoch 동기화 실패, IMU 오류를 각각 시험한다. 복구 뒤에는 이전
`cmd_vel`을 재사용하지 않으며 새로운 유효 명령을 받아야만 주행할 수 있어야
한다. 상세 Pi 검증 명령과 rosbag 절차는
[Raspberry Pi ROS 통합 가이드](rpi-ros-integration.md)에 정의한다.
