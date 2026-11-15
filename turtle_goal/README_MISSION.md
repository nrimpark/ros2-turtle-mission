# 장애물 회피 + 스모그 대응 turtlesim 실습

기존 `turtle_goal` 패키지에 별도 실행 파일 3개와 launch 파일을 추가한다.
기존 `goal_publisher`, `turtle_controller`, `move_to_goal.launch.py`는 수정하지 않는다.
새 예제와 기존 예제는 같은 토픽을 사용하므로 한 번에 하나만 실행한다.

## 구현한 시나리오

1. turtlesim 기본 출발점(약 5.54, 5.54)에서 목표 (8, 8)로 이동한다.
2. (6.5, 6.5)의 정지한 두 번째 거북이를 고정 장애물 표시로 사용한다.
3. 목표까지의 선분이 장애물 안전영역을 지나면 경유점을 계산하고 우회한다.
4. y 좌표 7.0~7.7의 가상 스모그 구간에 들어가면 감속한다.
5. 진입 1.2초 후 가상 시야 신호를 2초간 0으로 바꾼다. 이때 제어 노드는 정지한다.
6. 신호가 회복되면 기존 경로로 저속 주행을 재개하고, 구간을 벗어나면 정상 속도로 복귀한다.
7. 목표까지의 거리가 0.1 미만이면 정지한다.

스모그 정지는 시야 신호에 의해 결정된다. 2초 후 회복은 환경 노드가 만들어 내는 **실습용 사건**이며, 실제 환경이 2초 후 안전해진다고 가정하는 알고리즘이 아니다.
시간 계산을 이용해 신호를 회복하므로 거북이가 멈춘 상태에서도 재출발할 수 있다.
스모그는 화면에 연기로 그려지지 않는다. 터미널 상태와 `/vision_confidence` 토픽으로 확인한다.

## 국방로봇경진대회 과제와의 연결

| 대회에서 검토한 동작 | 이번 실습의 대응 |
|---|---|
| 주행 중 장애물 대응 | 알려진 좌표의 원형 장애물 안전영역을 피하는 경유점 생성 |
| 스모그에서 최근 확인 경로 사용 | 시야 저하 시 이미 계산한 경로를 보존하고 저속 추종 |
| 시야 부족 시 감속·정지 | 신뢰도 0.4일 때 감속, 0.0일 때 정지 |
| 위치 추정을 이용한 주행 | turtlesim의 위치 토픽을 사용해 제어 흐름을 모의 |
| 환경 회복 후 임무 재개 | 신호가 회복되면 재출발하고 최종 목표에 도착 |

카메라·BEV·IMU·엔코더를 구현한 것이 아니다. `/turtle1/pose`는 시뮬레이터가 제공하는 정답 위치이며, 시야 상태는 환경 노드가 생성하는 가상 신호이다.
실제 카메라 장애물 인식이나 충돌 물리도 구현하지 않았다. turtlesim의 거북이들은 서로 통과할 수 있으므로 회피는 직접 작성한 제어 계산으로 수행한다.
제어는 거리/방향 오차에 대한 비례 제어이며 RPP는 아니다. 실습 범위는 고정 원형 장애물 하나이다.

## 노드와 토픽

| 노드 | 구독 | 발행 | 역할 |
|---|---|---|---|
| `/mission_goal_publisher` | — | `/goal_position` (Point) | 목표 (8, 8) 전달 |
| `/mission_environment` | `/turtle1/pose`, `/obstacle/pose` (Pose) | `/obstacle_position` (Point), `/vision_confidence` (Float32) | 가상 장애물·시야 신호 전달, `/spawn`으로 표시용 거북이 생성 |
| `/mission_controller` | `/turtle1/pose`, `/goal_position`, `/obstacle_position`, `/vision_confidence` | `/turtle1/cmd_vel` (Twist), `/mission_state` (String) | 경유점 생성·주행·감속·정지 |
| `/turtlesim` | `/turtle1/cmd_vel` | `/turtle1/pose`, `/obstacle/pose` | 위치 갱신 및 화면 표시 |

직접 작성한 노드는 3개이고 turtlesim까지 총 4개이다. 두 번째 거북이는 별도 ROS 노드가 아니라 turtlesim 내부의 표시 객체이다.
장애물 Point의 z는 0이며, 반지름은 `mission_logic.py`의 `OBSTACLE_RADIUS=0.45`로 설정한다.
안전 중심 거리는 장애물 반지름 0.45 + 거북이 반지름 0.25 + 여유 거리 0.20 = 0.90이다.
계획할 때는 경유점 도달 오차와 제어 주기를 고려해 0.20의 여유를 추가한다. 단위는 실습용 turtlesim 좌표이다.

## 설치와 실행

Windows PowerShell이 아니라 Ubuntu 터미널에서 실행한다.
기존 launch/키보드 조종을 Ctrl+C로 종료하고 이전 거북이 창도 닫는다.

```bash
cd ~/ros2_ws
python3 install_turtle_mission.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtle_goal
source ~/ros2_ws/install/setup.bash
ros2 launch turtle_goal defense_mission.launch.py
```

설치 프로그램은 `~/ros2_ws/src/turtle_goal`에 파일을 추가한다. 수정하는 기존 파일과 같은 이름의 파일은 먼저 `~/ros2_ws/mission_backups/날짜_시간/`에 백업한다.
백업 폴더는 `src` 바깥이므로 colcon이 중복 패키지로 인식하지 않는다.
다시 시연할 때에는 Ctrl+C로 launch를 종료한 뒤 같은 launch 명령을 다시 실행한다.
진행 중 `/reset`만 호출하면 내부 임무 상태가 초기화되지 않으므로 재시연에는 launch 재시작을 사용한다.

## 확인할 로그

`AVOID_OBSTACLE` → `SLOW_FOG` → `STOP_VISION_LOST` → `SLOW_FOG` → `GO_TO_GOAL` → `ARRIVED`

경유점 통과 시점에 따라 `GO_TO_GOAL`이 중간에 추가로 나올 수 있다.
도착 후에도 노드들은 실행된 상태를 유지한다. `ARRIVED`가 보이고 거북이가 멈추면 성공이다.
목표는 반복 발행하지만 매번 로그를 출력하지 않는다.

별도의 Ubuntu 터미널에서 아래와 같이 확인할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /mission_state
```

토픽 확인을 마친 뒤 Ctrl+C를 누르고 `rqt_graph`를 실행하면 통신 구조를 확인할 수 있다.
`STOP_MISSING_DATA`는 입력이 아직 없거나 오래된 상태이다. 잠깐 나타난 뒤 다른 상태로 바뀌는 것은 정상이다.
필요한 위치·장애물·시야 정보가 1초 이상, 목표 정보가 3초 이상 도착하지 않으면 0 속도를 발행한다.
이것은 제어 노드가 동작 중일 때의 방어 동작이며 실제 로봇의 독립적인 비상 정지 장치가 아니다.

## 보고서에 넣을 설명

본 실습에서는 국방로봇경진대회에서 검토한 장애물 대응과 스모그 구간 주행을 단순화하여 ROS 2 turtlesim 환경에 구현하였다. 목표 전달 노드, 환경 정보 전달 노드, 주행 제어 노드를 직접 작성하고 하나의 launch 파일로 turtlesim과 함께 실행하도록 구성하였다. 제어 노드는 목표 좌표와 현재 위치를 비교하며, 알려진 고정 장애물의 안전영역과 경로가 겹치는 경우 경유점을 계산하여 우회한다. 가상 스모그 구간에서는 시야 신뢰도에 따라 감속 또는 정지하고, 신호가 회복되면 이미 확인한 경로를 따라 주행을 재개한다. 최종 목표와의 거리가 설정된 허용 오차 이내이면 정지한다. 실제 센서 인식 대신 가상 환경 신호와 시뮬레이터의 위치 정보를 사용하여 노드 간 통신과 상황별 제어 전환을 확인하는 것을 목적으로 하였다.

실제 실행 결과와 rqt_graph 캡처는 본인 WSL에서 확인한 뒤 보고서에 추가한다.

## 참고

- [ROS 2 Jazzy: turtlesim, ros2, rqt](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [ROS 2 Jazzy: Python launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

위 문서는 ROS 2 인터페이스 사용법의 참고이며, 장애물·스모그 시나리오와 제어 알고리즘은 이 실습을 위해 작성하였다.
