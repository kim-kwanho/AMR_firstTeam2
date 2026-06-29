# AMR_firstTeam2 — Toy Guide 실기(Storagy) 배포 워크스페이스

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](.venv)

**Toy Guide** 팀 프로젝트([AMR_firstTeam1](https://github.com/kim-kwanho/AMR_firstTeam1))를 **Storagy 실하드웨어 AMR** 위에서 돌리기 위한 ROS 2 워크스페이스입니다.

> 시뮬레이션 개발·검증은 [AMR_firstTeam1](https://github.com/kim-kwanho/AMR_firstTeam1) (Docker + Gazebo)에서 진행하고,  
> 이 레포는 동일한 애플리케이션 패키지를 **실기 환경에 맞게 통합·배포**하는 용도입니다.

---

## firstTeam1과의 관계

| 항목 | [AMR_firstTeam1](https://github.com/kim-kwanho/AMR_firstTeam1) | **AMR_firstTeam2** (이 레포) |
|------|---------------------------------------------------------------|------------------------------|
| 목적 | 시뮬레이션 개발·데모 | **Storagy 실로봇 onboard 실행** |
| 실행 환경 | Docker Compose + noVNC | 네이티브 ROS 2 Humble (로봇 PC 또는 SSH) |
| 시뮬레이션 | Gazebo Harmonic (`full_bringup`) | 선택적 (로컬 PC에서만) |
| 하드웨어 드라이버 | 없음 | LiDAR, Orbbec 카메라, MCU 모터 |
| 웹 대시보드 | `:8090` | `:8091` (로봇 기본 `storagy_program_ws`와 포트 분리) |
| 워크스페이스 형태 | `src/` + Docker 마운트 | `colcon` ROS 2 워크스페이스 전체 |

**코드 동기화 흐름**

```text
AMR_firstTeam1 (시뮬 검증)
        │
        │  storagy, storagy_llm, storagy_hide, storagy_guide, storagy_interfaces
        ▼
AMR_firstTeam2 (실기 배포 + 하드웨어 드라이버)
        │
        ▼
Storagy AMR onboard (motor_driver2, sick_scan2, Orbbec …)
```

시뮬에서 검증한 런치·파라미터·FSM 로직을 이 워크스페이스로 가져온 뒤, 실기 전용 bringup(`bringup.launch.py`, `amr_web_bringup.launch.py`)으로 통합합니다.

---

## 프로젝트 개요

평상시 구석 대기석에 은폐(Freeze)되어 있다가, 안내 임무 교대(Take-over) 후 Nav2 자율주행·YOLO 인식·LLM 명령으로 손님을 안내하고 복귀하는 **시각장애인 안내 보조 AMR** 시스템입니다.

| 단계 | 동작 | 담당 패키지 |
|------|------|-------------|
| 1. 위장·대기 | ArUco 도킹, LED/OLED OFF, 모터 잠금 | `storagy_hide` |
| 2. 임무 교대 | LLM/웹 명령 → 입구 Nav2 주행 | `storagy_llm`, `storagy_guide` |
| 3. 안내 주행 | YOLO 점자 블록 추종 + Nav2 | `storagy`, `storagy_guide` |
| 4. 회피·복귀 | 사람 감지 Freeze → 재개 → 대기석 복귀 | `storagy_hide` |

---

## 패키지 구성

### Toy Guide 애플리케이션 (firstTeam1과 공유)

| 패키지 | 역할 |
|--------|------|
| `storagy` | URDF, 맵, Nav2·SLAM 런치, 시뮬/실기 bringup |
| `storagy_interfaces` | 커스텀 srv (`Agent`, `SetLamp`, `Emotion`) |
| `storagy_llm` | LLM 에이전트, CLI, Flask 웹 대시보드 |
| `storagy_hide` | 숨는팀 FSM, ArUco 도킹, YOLO 연동, 동적 코스트맵 |
| `storagy_guide` | 안내팀 Nav2 미션 FSM |

### Storagy 실기 전용

| 패키지 | 역할 |
|--------|------|
| `motor_driver2` | MCU 시리얼 — 모터·오도메트리·LED 제어 |
| `sick_scan2-master` | SICK 2D LiDAR (`/scan`) |
| `OrbbecSDK_ROS2` | Orbbec 깊이/RGB 카메라 |
| `path_evaluator` | Nav2 경로 평가 유틸 |

> `amr-2026/` 은 중첩 개발용 워크스페이스(Nav2 소스 사본 등)이며, **메인 `colcon build` 대상이 아닙니다.**

---

## 사전 요구사항

- **ROS 2 Humble** (Ubuntu 22.04, Storagy onboard 또는 동일 환경 WSL)
- Nav2, `slam_toolbox`, `robot_state_publisher`, `rviz2`
- 시뮬 로컬 테스트 시: Gazebo Sim (`ros_gz_sim`, `ros_gz_bridge`)
- Python 3.10+ 및 워크스페이스 루트 `.venv` (LLM/YOLO 노드)
- 워크스페이스 루트 `yolov8n.pt` (git 제외, 로컬 배치)
- OpenAI API 키 (`src/.env`, git 제외)

---

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

> `src/.env`와 `yolov8n.pt`는 `.gitignore`에 포함됩니다. API 키를 커밋하지 마세요.

### 4. YOLO 모델

Ultralytics에서 `yolov8n.pt`를 받아 워크스페이스 **루트**에 둡니다.

### 5. 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 실행 (Storagy 실기)

로봇 onboard 또는 SSH 세션에서 워크스페이스를 소스한 뒤 실행합니다.

### 통합 bringup (권장)

Nav2 + 하드웨어(LiDAR·카메라·모터) + YOLO + LLM + 웹 대시보드를 한 번에 기동합니다.

```bash
source install/setup.bash
ros2 launch storagy amr_web_bringup.launch.py
```

- 웹 대시보드: http://\<로봇 IP\>:8091  
  (Storagy 기본 `storagy_program_ws`가 8090을 쓰므로 이 워크스페이스는 **8091** 사용)

### 베이스만 (Nav2 + 센서 + 모터)

```bash
ros2 launch storagy bringup.launch.py
```

### 모터·상태만 (텔레옵·점검)

```bash
ros2 launch storagy teleop_bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 별도 터미널
```

### 숨는팀 / 안내팀 (베이스 위에 추가)

```bash
# 실하드웨어 MCU 연동
ros2 launch storagy_hide hide_bringup.launch.py use_sim:=false

# 안내팀 Nav2 미션
ros2 launch storagy_guide guide_nav.launch.py
```

### LLM 대화형 CLI

```bash
ros2 run storagy_llm agent_client
```

### SLAM / 맵핑

```bash
ros2 param set /amcl tf_broadcast false
ros2 launch storagy cartographer.launch.py
```

---

## 실행 (로컬 시뮬 — 개발·회귀 테스트)

실기 없이 PC에서 Gazebo 시뮬을 돌릴 때는 firstTeam1 Docker 환경을 권장합니다.  
이 워크스페이스에서도 `full_bringup`으로 동일 스택을 띄울 수 있습니다.

```bash
ros2 launch storagy full_bringup.launch.py
ros2 launch storagy full_bringup.launch.py enable_hide:=false
```

---

## 로봇 접속 (참고)

Storagy 실하드웨어 SSH·맵핑 절차는 [AMR_firstTeam1 — README_realrobot.md](https://github.com/kim-kwanho/AMR_firstTeam1/blob/main/README_realrobot.md)를 참고하세요.

일반적인 onboard 경로:

```bash
cd ~/AMR_firstTeam2_ws   # 또는 로봇에 배포한 경로
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 주요 토픽

| 토픽 | 설명 |
|------|------|
| `/scan` | SICK LiDAR |
| `/camera/color/image_raw` | Orbbec RGB |
| `/cmd_vel`, `/odom` | 속도 명령, 오도메트리 |
| `/yolo/detected_image` | YOLO 검출 결과 |
| `/hide/state` | 숨는팀 FSM 상태 |
| `/guide/command`, `/guide/state` | 안내팀 명령·상태 |

---

## 팀 역할

| 팀 | 패키지 | FSM / 역할 |
|----|--------|------------|
| 숨는팀 | `storagy_hide` | FREEZE → WAKE → GUIDE → RETURN → DOCK |
| 안내팀 | `storagy_guide` | 손님 도착 시 목표 지점 안내 |
| LLM | `storagy_llm` | 장소 안내, 감정 표현, 미션 연동 |

---

## 개발 워크플로우

1. **시뮬**: [AMR_firstTeam1](https://github.com/kim-kwanho/AMR_firstTeam1) Docker에서 기능 개발·검증  
2. **동기화**: `storagy*`, `storagy_llm`, `storagy_hide`, `storagy_guide` 변경분을 이 워크스페이스로 반영  
3. **실기**: 로봇에서 `colcon build` 후 `amr_web_bringup.launch.py`로 통합 테스트  
4. **숨는팀 실기**: `hide_bringup.launch.py use_sim:=false`로 MCU·ArUco 연동 확인

---

## 관련 저장소

| 저장소 | 설명 |
|--------|------|
| [kim-kwanho/AMR_firstTeam1](https://github.com/kim-kwanho/AMR_firstTeam1) | Toy Guide 시뮬레이션 개발 (Docker) |
| [bluephysi01/storagy-simulation-system-docker](https://github.com/bluephysi01/storagy-simulation-system-docker) | 시뮬레이션 베이스 upstream |

---

## 라이선스

패키지별 `package.xml` 및 서드파티 README를 참고하세요.
