# 엔코더와 주행 제어 보정

모든 첫 시험은 차체를 고정하고 바퀴를 지면에서 띄운 상태에서 수행한다.
기본값은 미보정 상태이며 폐루프 주행을 허용하지 않는다.

## 1. 엔코더 CPR과 방향

`ENCODER_CPR`은 모터축이 아니라 바퀴가 한 바퀴 돌 때의
PCNT count다. 현재 드라이버는 A/B상의 모든 edge를 사용하는 quadrature
x4 계수이므로 다음처럼 계산하거나 직접 측정한다.

```text
바퀴 1회전 count = 엔코더 PPR × 4 × 감속비
```

바퀴에 기준선을 표시하고 정확히 10회전시킨 뒤 누적 count의 절댓값을
10으로 나누면 오차를 줄일 수 있다. 전진으로 돌렸을 때 네 바퀴 count가
모두 양수여야 한다. 특정 바퀴만 음수이면 `ENCODER_FRONT_LEFT_INVERTED`,
`ENCODER_FRONT_RIGHT_INVERTED`, `ENCODER_REAR_LEFT_INVERTED` 또는
`ENCODER_REAR_RIGHT_INVERTED` 중 해당 설정을 변경한다.

GPIO34/35/36/39에는 내부 풀업이 없으므로 해당 엔코더가 오픈 컬렉터라면
반드시 3.3 V 외부 풀업을 사용한다. 정지 중 count가 변하면 배선, 접지,
외부 풀업과 `ENCODER_GLITCH_FILTER_NS`를 먼저 확인한다.

## 2. 차체 기하와 최대 RPM

- `DRIVE_WHEEL_RADIUS_MM`: 접지된 상태에서 측정한 유효 바퀴 반지름
- `DRIVE_TRACK_WIDTH_MM`: 좌우 바퀴 접촉면 중심 사이 거리
- `DRIVE_MAX_WHEEL_RPM`: 배터리 정상 전압에서 측정한 안전한 최대 RPM

무부하 지름만 사용하는 것보다 차량을 바닥에서 직진시켜 실제 이동 거리로
유효 반지름을 보정하는 편이 정확하다. 제자리 회전 명령 후 실제 yaw가
작으면 윤거 값을 줄이고, 실제 yaw가 크면 윤거 값을 늘려 조정한다.

## 3. PID

Kconfig의 PID 값은 실제 계수에 1000을 곱한 정수다. 예를 들어 Kp 0.25는
`DRIVE_PID_KP_MILLI=250`으로 입력한다.

1. Ki와 Kd를 0으로 두고 Kp를 작은 값부터 올린다.
2. 목표 RPM에 빠르게 접근하면서 지속 진동하지 않는 Kp를 선택한다.
3. 정상상태 속도 오차가 있으면 Ki를 조금씩 추가한다.
4. overshoot나 빠른 진동 억제가 필요할 때만 작은 Kd를 추가한다.
5. 전진, 후진, 좌·우 회전과 서로 다른 배터리 전압에서 다시 확인한다.

폐루프 태스크는 anti-windup을 적용하며 출력은 정규화된 `-1000..1000`으로
제한한다. 폐루프 경로에서는 블로킹 기동 부스트를 사용하지 않는다.

## 4. MPU6050

부팅 시 기본 200개 표본으로 정지 바이어스를 계산한다. 이 시간 동안 차체를
평평한 곳에 완전히 정지시킨다. 중력 방향의 가속도 1 g는 유지하고 나머지
정적 오프셋과 gyro 바이어스를 제거한다. 보정 중 차량을 움직였다면 전원을
다시 넣어 재보정한다.

`MPU6050_CALIBRATION_RATE_HZ`는 시작 보정 표본의 간격이며 센서 내부
output-data rate 설정이 아니다. 현재 Registry 드라이버는 내부 sample
rate와 DLPF를 변경하지 않는다.

## 5. 최종 확인

설정 후 `/joint_states`, `/odom`, `/diagnostics`를 확인한다. 명령을 끊었을
때 500 ms 이내 정지하고 `vehicle_ecu/drive` diagnostics가 command timeout
`WARN`을 보고해야 한다. 바퀴를 잡은 상태에서 명령했을 때도 stall timeout
후 출력이 정지하고 drive diagnostics가 `ERROR`를 보고해야 한다.
