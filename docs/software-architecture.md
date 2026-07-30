# 소프트웨어 구조

펌웨어는 하드웨어 접근, 제어 정책, ROS 통신을 분리한다.

```text
Raspberry Pi 5 / ROS 2 Jazzy
robot_state_publisher / EKF / upper applications
           |
  micro_ros_agent
           |
     USB-UART / XRCE-DDS
           |
       ros_bridge
      /           \
     drive          imu
    /     \
wheel_encoder motor
       \       /
          board

Registry drivers:
  espp/encoder
  espressif/bdc_motor

Project bounded-I2C driver:
  imu/MPU6050
```

ESP32는 센서 취득, 주행 제어와 휠 기반 raw odometry를 담당한다. Raspberry
Pi는 URDF, IMU·휠 odometry 융합과 `odom → base_link` 동적 TF를 담당한다.
ECU는 TF를 발행하지 않아 Pi의 EKF가 해당 변환의 단일 권한자가 된다.
상세 ECU 계약은 [micro-ROS ECU 설계](microros-design.md), Pi 패키지와
센서 융합 구성은 [Raspberry Pi ROS 통합 가이드](rpi-ros-integration.md)를
참조한다.

## 컴포넌트 책임

| 컴포넌트 | 책임 |
|---|---|
| `board` | 차량 GPIO와 Kconfig 값을 각 드라이버 설정 구조체로 변환 |
| `motor` | Registry BDC 드라이버 연결, 네 모터 동기화, skid-steer mixing과 정지 정책 |
| `wheel_encoder` | Registry PCNT 드라이버 연결, 좌·우 누적 count와 RPM 계산 |
| `imu` | 유한-timeout MPU6050 I2C, 독립 취득 태스크, snapshot, SI 단위 환산과 복구 |
| `drive` | 50 Hz 좌우 PID, twist 변환, odometry와 fail-safe |
| `ros_bridge` | micro-ROS 엔티티, UART transport, 재접속과 ROS 시각 동기화 |

`ros_bridge` 내부에서 `ros_diagnostics` 모듈은 fault snapshot을 표준
`DiagnosticArray`로 변환하고 문자열 저장공간을 연결 시점에 미리 할당한다.
`ros_messages` 모듈은 `drive`와 `imu` snapshot을 ROS 메시지로 변환하고
frame, covariance, sequence 저장공간을 소유한다.
세션 태스크는 연결 상태 전이와 발행 주기만 조정하며 diagnostics의 상태
판정 규칙이나 필드 변환 규칙을 직접 소유하지 않는다.

`espressif/bdc_motor`와 `espp/encoder`가 범용 하드웨어 접근을 담당한다.
MPU6050은 모든 I2C transaction의 반환 시간을 제한하기 위해 프로젝트
`imu` 컴포넌트가 ESP-IDF master API를 직접 사용한다. 프로젝트 컴포넌트에는
차량 설정과 공개 C API, 단위 변환 및 안전 정책만 둔다.
PID, 센서 단위 환산, 엔코더 RPM 계산처럼 하드웨어와 무관한 계산은 각
컴포넌트의 별도 `*_math.c` 모듈과 private 헤더에 둔다. 따라서 테스트
가능성을 위해 내부 계산 함수를 제품 공개 API에 노출하지 않는다.

## 실행 흐름

`app_main()`은 GPIO 할당 검증 직후 micro-ROS와 IMU 취득 태스크를 시작하고
이어서 모터와 엔코더를 초기화한다. 모터 또는 엔코더가 준비되지 않으면
주행 태스크를 시작하지 않고 diagnostics로 상태를 알린다. IMU의
초기화·보정·복구는 전용 태스크에서 수행하므로 센서가 없거나 I2C가
timeout되어도 ROS executor와 wheel odometry는 계속 동작한다.

주행 제어 태스크는 기본 20 ms 주기로 엔코더를 읽고 좌우 PID 출력을 네
모터에 적용한다. `cmd_vel`을 500 ms 동안 받지 못하거나, 목표 속도가
있는데 엔코더 변화가 500 ms 동안 없으면 모터를 정지한다. ROS Agent 연결이
끊겨도 즉시 주행 명령을 해제한다.

주행 반경, 윤거, 최대 RPM, 엔코더 CPR 중 하나라도 0이면
`DRIVE_FAULT_NOT_CALIBRATED`가 유지되며 모터 명령을 거절한다. 따라서 저장소
기본 설정으로 실차가 뜻하지 않게 움직이지 않는다.

## 공개 인터페이스

- `imu_get_snapshot()`은 마지막 가속도 m/s², 각속도 rad/s, 온도 °C와
  유효성·상태·마지막 성공 시각을 non-blocking으로 반환한다.
- `wheel_encoder_sample_all()`은 누적 count, 표본간 delta, RPM을 반환한다.
- `drive_set_twist()`는 선속도 m/s와 yaw 각속도 rad/s를 받는다.
- `drive_get_state()`는 바퀴와 odometry 상태 및 fault 비트를
  snapshot으로 반환한다.

ROS 태스크는 하드웨어를 직접 읽지 않는다. IMU와 drive snapshot이 100 ms
이상 오래되거나 invalid이면 해당 센서 토픽만 중단하며 diagnostics, 다른
토픽과 명령 처리는 계속한다.

컴포넌트 간 핀 상수 복사를 피하기 위해 모든 보드 설정은
`components/board`에 모았다. 수치 계산 함수는 하드웨어 없이 Unity
테스트에서 검증할 수 있도록 순수 함수로 유지한다.
