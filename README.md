# AMR_firstTeam2

Storagy AMR 기반 ROS 2 워크스페이스입니다. **Toy Guide** 데모(교실 안내 + 은폐/복귀)를 위해 자율주행, LLM 에이전트, YOLO 사람 감지, 숨는팀/안내팀 FSM을 통합합니다.

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `storagy` | 로봇 URDF/메시, Gazebo Sim 월드, Nav2·SLAM 런치, 시뮬/실기 bringup |
| `storagy_llm` | LLM 에이전트, CLI, Flask 웹 대시보드 (`http://localhost:8091`) |
| `storagy_hide` | 숨는팀 FSM, ArUco 도킹, YOLO 연동, 동적 코스트맵 |
| `storagy_guide` | 안내팀 Nav2 미션 FSM |
| `storagy_interfaces` | 커스텀 srv (`Agent`, `SetLamp`, `Emotion`) |
| `motor_driver2` | MCU 시리얼 모터/오도메트리/LED 제어 |
| `path_evaluator` | Nav2 경로 평가 유틸 |
| `sick_scan2-master` | SICK 2D LiDAR 드라이버 |
| `OrbbecSDK_ROS2` | Orbbec 깊이 카메라 드라이버 |
| `amr-2026/` | 중첩 개발용 워크스페이스 (Nav2 소스 사본 등). 메인 빌드 대상 아님 |

## 사전 요구사항

- **ROS 2 Humble**
- Nav2, Gazebo Sim (`ros_gz_sim`, `ros_gz_bridge`), `slam_toolbox`, `robot_state_publisher`, `rviz2`
- Python 3.10+ 및 워크스페이스 루트 `.venv` (LLM/YOLO 노드용)
- 워크스페이스 루트에 `yolov8n.pt` (git 제외, 로컬 배치)
- OpenAI API 키 (`src/.env`, git 제외)

## 환경 설정

### 1. 저장소 클론

```bash
git clone git@github.com:kim-kwanho/AMR_firstTeam2.git AMR_firstTeam2_ws
cd AMR_firstTeam2_ws
```

### 2. Python venv

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install langchain langchain-openai langchain-core langgraph flask python-dotenv \
  ultralytics torch torchvision opencv-python pyyaml
```

### 3. API 키

```bash
cat > src/.env <<'EOF'
OPENAI_API_KEY=your-key-here
EOF
```

> `src/.env`와 `yolov8n.pt`는 `.gitignore`에 포함되어 있습니다. API 키를 커밋하지 마세요.

### 4. YOLO 모델

Ultralytics에서 `yolov8n.pt`를 받아 워크스페이스 **루트**에 둡니다.

```bash
# 예: ultralytics CLI 사용
yolo export model=yolov8n.pt  # 또는 직접 다운로드
```

## 빌드

```bash
cd ~/AMR_firstTeam2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 실행

### 시뮬레이션 (통합)

Gazebo + Nav2 + 배회 + YOLO + LLM + 웹 + 숨는팀 FSM을 한 번에 실행합니다.

```bash
source install/setup.bash
ros2 launch storagy full_bringup.launch.py
```

숨는팀 FSM 없이 실행:

```bash
ros2 launch storagy full_bringup.launch.py enable_hide:=false
```

LLM 대화형 CLI (별도 터미널):

```bash
ros2 run storagy_llm agent_client
```

### 실기

```bash
ros2 launch storagy bringup.launch.py
ros2 launch storagy amr_web_bringup.launch.py   # YOLO + LLM + 웹 포함
ros2 launch storagy teleop_bringup.launch.py    # 모터/상태만
```

### 숨는팀 / 안내팀 (베이스 위에 추가)

```bash
ros2 launch storagy_hide hide_bringup.launch.py
ros2 launch storagy_hide hide_bringup.launch.py use_sim:=false
ros2 launch storagy_guide guide_nav.launch.py
```

### SLAM / 맵핑

```bash
ros2 launch storagy cartographer.launch.py
ros2 launch storagy cartographer_sim.launch.py
```

## 주요 토픽

| 토픽 | 설명 |
|------|------|
| `/scan` | LiDAR |
| `/camera/color/image_raw` | RGB 카메라 |
| `/cmd_vel`, `/odom` | 속도 명령, 오도메트리 |
| `/yolo/detected_image` | YOLO 검출 결과 |
| `/hide/state` | 숨는팀 FSM 상태 |

## 팀 역할

- **숨는팀** (`storagy_hide`): FREEZE → WAKE → GUIDE → RETURN → DOCK
- **안내팀** (`storagy_guide`): 손님 도착 시 목표 지점 안내
- **LLM** (`storagy_llm`): 장소 안내, 감정 표현, 미션 연동

## 라이선스

패키지별 `package.xml` 및 서드파티 README를 참고하세요.
