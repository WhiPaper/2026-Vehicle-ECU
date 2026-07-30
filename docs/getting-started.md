# 개발 시작 가이드

이 문서는 저장소를 처음 받은 개발자가 펌웨어를 빌드하고 ESP32에
플래시하기까지의 최소 절차를 설명한다. 실차 배선과 보정은 각각
[하드웨어 배선](hardware-wiring.md)과 [보정 가이드](calibration.md)를
따른다.

## 준비물

- ESP32 DevKitC V4 (ESP32-WROOM-32D)
- 데이터 통신을 지원하는 USB 케이블
- VS Code와 Dev Containers 확장 또는 호환되는 ESP-IDF 6.0.2 개발 환경
- 실차 시험 시 외부 모터 전원, 퓨즈와 비상 정지 수단

프로젝트와 CI가 고정한 ESP-IDF 버전은 `6.0.2`다. 다른 ESP-IDF 버전에서는
빌드 결과와 Component Manager 의존성이 달라질 수 있다.

## 권장 방법: devcontainer

저장소를 VS Code에서 연 뒤 **Dev Containers: Reopen in Container**를
실행한다. 컨테이너 생성이 끝나면 ESP-IDF 환경, Python 개발 도구와
pre-commit hook이 준비되고 프로젝트가 자동으로 reconfigure된다.

```bash
idf.py set-target esp32
idf.py build
```

첫 빌드는 Registry 컴포넌트와 micro-ROS 소스를 내려받고 정적 라이브러리를
만들기 때문에 이후 빌드보다 오래 걸린다. 해결된 버전과 해시는
`dependencies.lock`에 기록되며 `managed_components/`와 `build/`는
생성물이므로 수정하거나 커밋하지 않는다.

## 로컬 ESP-IDF 환경

ESP-IDF 환경을 활성화한 터미널에서 Python 도구를 설치한 뒤 빌드한다.

```bash
python -m pip install --upgrade pip==26.1.2
python -m pip install -r requirements-dev.txt
idf.py set-target esp32
idf.py build
```

`idf.py: command not found`가 나오면 먼저 사용하는 ESP-IDF의
`export.sh`를 source한다. 빌드 전에 `python -m pip check`로 Python
의존성 충돌을 확인할 수 있다.

## 설정과 안전 잠금

저장소 기본값에서는 다음 실측 항목이 0이므로 폐루프 주행이 잠겨 있다.

- `ENCODER_CPR`
- `DRIVE_WHEEL_RADIUS_MM`
- `DRIVE_TRACK_WIDTH_MM`
- `DRIVE_MAX_WHEEL_RPM`
- `DRIVE_PID_KP_MILLI`, `DRIVE_PID_KI_MILLI`, `DRIVE_PID_KD_MILLI`

값을 추측해 잠금을 해제하지 않는다. 차체를 고정하고 바퀴를 띄운 상태에서
[보정 절차](calibration.md)를 수행한 뒤 설정한다.

```bash
idf.py menuconfig
idf.py build
```

`sdkconfig.defaults`는 새 빌드 디렉터리의 기준값이고, 이미 생성된
`sdkconfig`가 있으면 그 값이 우선한다. 기본값 변경을 확인하려면 기존
설정과의 차이를 검토해야 한다.

## 플래시

USB 장치를 찾고 펌웨어를 기록한다.

```bash
ls -l /dev/serial/by-id/
idf.py -p /dev/serial/by-id/<CP2102-device> flash
```

UART0은 921600 baud micro-ROS 전송 전용이며 일반 애플리케이션 콘솔은
비활성화되어 있다. 플래시 전에 같은 장치를 사용 중인 micro-ROS Agent나
serial monitor를 종료한다. 플래시 후 동작 확인은
[Raspberry Pi 5 micro-ROS 연결](microros-rpi5.md)을 따른다.

## 변경 전 기본 검증

```bash
pre-commit run --all-files
idf.py build
```

컴포넌트 단위·하드웨어 시험과 CI 범위는 [시험 가이드](testing.md)에
정리되어 있다.
