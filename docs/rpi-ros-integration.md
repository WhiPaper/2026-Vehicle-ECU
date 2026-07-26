# Raspberry Pi ROS 통합 가이드

이 문서는 Raspberry Pi 5에서 ECU를 사용하는 ROS 2 Jazzy 패키지를 설계하고
개발하기 위한 출발점이다. ECU의 wire 계약은
[micro-ROS ECU 설계](microros-design.md), Agent 실행과 serial 문제 해결은
[Raspberry Pi 5 micro-ROS 연결](microros-rpi5.md)이 기준이다.

Pi용 ROS 패키지는 이 ECU 저장소에 두지 않고 별도 저장소에서 구현한다.
아래 경로와 코드는 해당 저장소를 구성할 때 사용할 권장 구조와 참고
예시이며, 이 저장소에서 직접 빌드되는 산출물이 아니다.

## 책임과 권장 workspace

```text
src/
  vehicle_bringup/
    launch/vehicle.launch.py
    config/ekf.yaml
  vehicle_description/
    urdf/vehicle.urdf.xacro
```

- `vehicle_bringup`은 micro-ROS Agent, `robot_state_publisher`,
  `robot_localization` EKF와 remap을 조합한다.
- `vehicle_description`은 `base_link`, `imu_link`, 바퀴 링크와 joint의
  기하를 소유한다.
- teleop, navigation, 기록 노드는 `cmd_vel`, `odometry/filtered`,
  diagnostics와 TF만 사용하고 serial 장치나 ECU 내부 fault bit에 직접
  의존하지 않는다.

Nav2, SLAM, 원격 teleop과 systemd 서비스 자체는 이 문서의 구현 범위 밖이다.

## Bringup 순서

1. `/dev/serial/by-id` 경로로 micro-ROS Agent를 실행한다.
2. `vehicle_ecu` 노드와 raw 토픽의 타입, QoS, timestamp를 확인한다.
3. `robot_state_publisher`를 시작해 URDF 고정 변환을 제공한다.
4. EKF를 시작해 raw wheel odometry와 IMU를 융합한다.
5. diagnostics가 주행 가능 상태이고 데이터가 fresh한 경우에만 상위 명령
   source를 허가한다.

Agent 실행 예시는 다음과 같다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vehicle_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/serial/by-id/<CP2102-device> -b 921600 -v4
```

## URDF와 TF 소유권

권장 TF tree는 다음과 같다.

```text
odom                         EKF가 동적으로 발행
└── base_link
    ├── imu_link             URDF fixed joint
    ├── left_wheel_link      robot_state_publisher + joint_states
    └── right_wheel_link     robot_state_publisher + joint_states
```

ECU는 TF를 발행하지 않는다. EKF의 `publish_tf`를 켜고 다른 odometry 또는
변환 노드에서는 `odom → base_link`를 발행하지 않는다. IMU의 xyz 위치와
회전은 실측하여 `base_link → imu_link` fixed joint에 기록한다. 축은 ROS
REP-103 기준 x 전방, y 좌측, z 위쪽으로 맞추고 센서 장착 방향이 다르면
URDF 회전으로 표현한다.

ECU가 보내는 joint 이름은 `left_wheel_joint`, `right_wheel_joint`다. 4륜
시각 모델이라면 같은 쪽 앞·뒤 joint를 별도의 보조 노드나 mimic 가능한
모델 정책으로 연결해야 한다. 기본 계약은 좌·우 대표 joint 두 개다.

## `robot_localization` 기준 설정

휠 odometry pose와 twist는 같은 엔코더에서 유도되므로 둘을 동시에 융합해
독립 측정처럼 취급하지 않는다. 기본 설정은 `odom`에서 body-frame
`linear.x`와 `angular.z`, IMU에서 `angular_velocity.z`만 사용한다.
MPU6050의 linear acceleration은 진동과 중력 제거 설정을 검증하기 전에는
융합하지 않는다.

`vehicle_bringup/config/ekf.yaml`의 시작점:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true
    print_diagnostics: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: odom
    odom0_config: [
      false, false, false,
      false, false, false,
      true,  false, false,
      false, false, true,
      false, false, false
    ]
    odom0_queue_size: 5
    odom0_differential: false

    imu0: imu/data_raw
    imu0_config: [
      false, false, false,
      false, false, false,
      false, false, false,
      false, false, true,
      false, false, false
    ]
    imu0_queue_size: 10
    imu0_differential: false
    imu0_remove_gravitational_acceleration: false
```

EKF 출력은 기본 `odometry/filtered`를 사용한다. 이 설정에서 MCU odometry의
yaw rate와 IMU yaw rate가 함께 융합된다. 실제 covariance를 측정하여 두
센서의 상대 신뢰도를 결정해야 한다. ECU는 Kconfig의 초기 covariance를
채우지만 실차 측정값으로 교체해야 한다.

## Launch 설계 예시

개발 launch는 다음 인자를 제공한다.

| 인자 | 기본값 | 의미 |
|---|---|---|
| `serial_device` | 필수 지정 | `/dev/serial/by-id/...` |
| `baudrate` | `921600` | ECU UART 속도 |
| `use_sim_time` | `false` | 실차에서는 false |

아래 코드는 패키지 구현 시 사용할 구조 예시다.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration("serial_device")
    baud = LaunchConfiguration("baudrate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("serial_device"),
        DeclareLaunchArgument("baudrate", default_value="921600"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "--dev", device, "-b", baud],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": "<xacro 결과를 launch에서 대입>"},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=["<vehicle_bringup>/config/ekf.yaml",
                        {"use_sim_time": use_sim_time}],
            remappings=[
                ("odom", "odom"),
                ("imu/data_raw", "imu/data_raw"),
            ],
            output="screen",
        ),
    ])
```

실제 구현에서는 `ament_index_python`으로 share directory를 찾고 xacro를
실행해 `robot_description`을 생성한다.

## QoS와 데이터 freshness

Pi subscriber는 ECU와 호환되는 QoS를 사용한다.

- `imu/data_raw`, `joint_states`, `odom`: sensor data profile과 같은
  best effort, volatile, keep last
- `cmd_vel`, `diagnostics`: reliable, volatile, keep last

`cmd_vel` publisher는 10 Hz 이상을 권장하며 ECU의 500 ms watchdog보다
충분히 빠르게 갱신한다. 상위 제어가 끝날 때 0 명령을 보내더라도 ECU
watchdog을 최종 안전장치로 유지한다.

상위 motion enable 조건은 최소한 Agent 연결, ECU drive diagnostics `OK`,
동기화된 시각, 200 ms 이내의 새 `odom`과 `imu/data_raw`다. Agent 또는 ECU
재시작 뒤에는 이전 명령을 자동 재생하지 않고 operator나 상위 state
machine이 새로 enable해야 한다. timestamp가 역행하거나 현재 ROS time보다
허용 범위를 벗어나면 해당 표본을 폐기하고 EKF를 재시작할지 운영 정책에
따라 결정한다.

현재 펌웨어는 표준 `diagnostic_msgs/msg/DiagnosticArray`를 발행한다. 이전
`std_msgs/msg/UInt32` 펌웨어와 현재 Pi 패키지는 같은 토픽 이름으로
혼용하지 않는다.

## 확인 명령

```bash
ros2 node list
ros2 topic list -t
ros2 topic info --verbose /imu/data_raw
ros2 topic info --verbose /odom
ros2 topic hz /imu/data_raw
ros2 topic hz /odom
ros2 topic delay /odom
ros2 topic echo /diagnostics
ros2 doctor --report
```

기본값은 IMU 50 Hz, odometry와 JointState 30 Hz, diagnostics 5 Hz다. 표준
메시지의 covariance 배열 때문에 직렬 부하가 높으므로 ECU 설계서의
transport budget 수용 조건을 먼저 통과해야 한다. 발행률을 바꾼 배포에서는
이 명령으로 실제 주기를 기록하고 EKF `frequency`와 `sensor_timeout`이 그
주기와 일치하는지 확인한다.

TF의 단일 소유권과 고정 변환을 확인한다.

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_tools view_frames
```

`odom → base_link`가 EKF 하나에서만 나오고, Agent 재연결 후에도 TF timestamp가
역행하지 않아야 한다.

짧은 안전 시험은 바퀴를 띄우고 보정이 끝난 차량에서 수행한다.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'
```

publisher를 종료하면 500 ms 안에 모터가 정지해야 한다.

## rosbag2 재현 시험

raw 입력, 융합 결과, 진단과 TF를 한 bag에 기록한다.

```bash
ros2 bag record \
  /imu/data_raw /joint_states /odom /odometry/filtered \
  /diagnostics /tf /tf_static
```

다음 시나리오를 별도 bag 또는 marker와 함께 기록한다.

| 시나리오 | 기대 결과 |
|---|---|
| Agent 없이 ECU 부팅 | 모터 정지, Agent 시작 후 엔티티 생성 |
| Agent 강제 종료 | 즉시 또는 watchdog 이내 정지, 오래된 명령 재사용 없음 |
| USB 분리·재연결 | 재연결과 epoch 재동기화, 새 명령 전까지 정지 |
| ECU 재부팅 | raw stream 일시 중단, EKF가 stale 입력을 사용하지 않음 |
| epoch 동기화 실패 | diagnostics WARN, 융합에서 비정상 stamp 거절 |
| IMU 오류 | wheel odometry 유지 가능, diagnostics WARN |
| 명령 timeout | 500 ms 안에 정지 |
| 엔코더 stall | 모터 정지, drive diagnostics ERROR |

재생 또는 분석 시 각 topic의 stamp가 단조 증가하는지, raw `odom`과
`odometry/filtered`에 비정상 위치 도약이 없는지, raw stream 중단 후 EKF가
오래된 값을 계속 출력하지 않는지 확인한다. bag 재생에서는
`use_sim_time=true`와 `ros2 bag play --clock`을 사용하며 실제 ECU와 Agent는
동시에 연결하지 않는다.
