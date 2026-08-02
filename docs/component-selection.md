# 컴포넌트 선정 기준

외부 컴포넌트는 ESP Component Registry에서 먼저 찾고, 다음 조건을 모두
만족할 때 로컬 구현을 대체한다.

1. 현재 하드웨어와 동작 방식이 일치한다.
2. ESP-IDF 6 및 대상 칩을 지원한다.
3. 기존의 단위, 오류 처리, 안전 정지 동작을 유지할 수 있다.
4. `idf_component.yml`과 `dependencies.lock`으로 버전을 재현할 수 있다.

## 적용 결과

| 기능 | 선택 | 프로젝트에 남긴 책임 |
| --- | --- | --- |
| 4채널 L9110S 출력 | `espressif/bdc_motor` 0.2.1 | `motor`의 4모터 자원 배치, 방향 반전, 시동 부스트, tank mixing과 안전 정지 |
| MPU6050 | ESP-IDF `i2c_master` 직접 사용 | 유한 transaction timeout, 독립 취득 태스크, SI 단위 변환, 정지 보정과 복구 |
| FL/FR/RL/RR quadrature 엔코더 | `espp/encoder` 1.1.6 | `wheel_encoder`의 C API, 바퀴별 반전, 표본간 delta, RPM과 timestamp |
| micro-ROS | Git commit 고정 | Registry 패키지가 없어 upstream ESP-IDF 컴포넌트를 Component Manager의 Git 의존성으로 사용 |

기존 로컬 `motor_hw`, `mpu6050`, `encoder` 저수준 드라이버는 제거했다.
이름이 겹치지 않도록 프로젝트 계층은 각각 `motor`, `imu`,
`wheel_encoder`로 단순화했다.

## 사용하지 않은 후보

| 후보 | 이유 |
| --- | --- |
| `espressif/mpu6050` 1.2.1 | upstream이 유지보수 종료를 명시했고 IDF 6에서 제거된 legacy I2C API를 사용한다. |
| `espressif/knob` | 사용자 노브 입력용 디코더이므로 고속 모터 quadrature 계수 용도와 다르다. |

`espp/encoder`는 `base_component`, `logger`, `format`을 전이 의존성으로
가져온다. 로컬 구현보다 flash 사용량은 늘지만, Registry 구현을 우선한다는
프로젝트 원칙에 따라 채택했다. 최종 firmware는 기본 app partition 안에
여유를 두고 빌드되는지 계속 확인한다.

Registry 의존성은 이를 직접 사용하는 `motor`, `wheel_encoder`, `imu`,
`ros_bridge` 컴포넌트의 `idf_component.yml`에 선언한다. 해결된 정확한
버전과 해시는 루트 `dependencies.lock`에 커밋한다.
`managed_components/`는 생성물이므로 저장소에 커밋하지 않는다.
