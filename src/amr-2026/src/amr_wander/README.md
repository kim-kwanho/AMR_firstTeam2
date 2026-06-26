# amr_wander

Storagy 로봇용 **경량 장애물 회피 주행(wandering) 노드**입니다. 테스트 용도로,
2D 라이다(`/scan`)만 보고 빈 공간을 향해 자유롭게 돌아다닙니다. 맵·로컬라이제이션·Nav2가
필요 없습니다.

- ROS 2 Humble / Python (`ament_python`)
- 입력: `/scan` (`sensor_msgs/LaserScan`)
- 출력: `/cmd_vel` (`geometry_msgs/Twist`)

## 동작 방식 (단순 반응형)

상태 2개(`FORWARD` ↔ `TURN`)로 동작합니다.

1. 전방 섹터(정면 ±`front_angle_deg`)의 최소 거리를 계산합니다. `inf`/`NaN`,
   범위를 벗어난 측정값은 무시합니다.
2. 전방 최소거리 > `safe_distance` → **직진** (`linear_speed`).
3. 전방이 막힘 → **정지 후 회전**. 좌측(+90°)과 우측(−90°) 섹터의 최소거리를
   비교해 **더 넓은 쪽으로 회전**하고, 전방이 다시 트일 때까지 그 방향을 유지합니다
   (방향을 매 틱마다 바꾸지 않아 떨림을 방지).

### 안전장치
- 스캔이 `scan_timeout`(기본 0.5s) 이상 들어오지 않으면 정지합니다.
- 노드 종료 시 `/cmd_vel`에 0을 발행해 로봇을 멈춥니다.

## 파라미터

| 이름 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `safe_distance` | `0.6` | m | 이보다 가까운 장애물이 전방에 있으면 회피 |
| `linear_speed` | `0.15` | m/s | 직진 속도 |
| `angular_speed` | `0.5` | rad/s | 회전 속도 |
| `front_angle_deg` | `30.0` | deg | 전방 섹터의 반각 |
| `scan_timeout` | `0.5` | s | 이 시간 동안 스캔이 없으면 정지 |
| `control_period` | `0.1` | s | 제어 루프 주기 |

## 빌드

```bash
cd ~/Desktop/amr-2026
colcon build --packages-select amr_wander
source install/setup.bash
```

## 실행

라이다(`sick_scan2`)와 모터 드라이버가 떠 있는 상태에서:

```bash
ros2 launch amr_wander wander.launch.py
```

노드만 직접 실행할 수도 있습니다:

```bash
ros2 run amr_wander wander
```

런타임에 파라미터를 바꾸려면:

```bash
ros2 param set /wander_node linear_speed 0.1
ros2 param set /wander_node safe_distance 0.8
```

## 토픽 이름이 다를 때

`/scan`·`/cmd_vel`과 다른 토픽을 쓰면 `launch/wander.launch.py`의 `remappings`만
수정하면 됩니다.

```python
remappings=[
    ('scan', '/your_scan_topic'),
    ('cmd_vel', '/your_cmd_vel_topic'),
],
```

## 주의

- 테스트용 반응형 컨트롤러라 좁은 통로·막다른 길에서는 갇히거나 진동할 수 있습니다.
- 처음에는 `linear_speed`를 낮게 두고 넓은 공간에서 시험하세요.
