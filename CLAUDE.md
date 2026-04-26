# ros2_3dslam_ws — Project Instructions

## 작업 실행 원칙

- **직접 실행 가능한 작업은 직접 실행하고 결과를 사용자에게 제시한다.**
  사용자에게 명령어를 안내하고 실행을 미루지 않는다. Bash, Read, Edit 등 가용한 도구로 수행 가능한 작업은 즉시 수행하고, 결과만 보고한다.
  단, 다음은 예외로 사용자 확인을 받는다:
  - 실행 중인 SLAM/매핑 데이터 손실 위험이 있는 프로세스 종료
  - 외부 시스템에 영향 (push, PR, 외부 메시지 발송 등)
  - 되돌리기 어려운 파일/브랜치 삭제

## 프로젝트 구조

- ROS2 Humble + Ignition Gazebo Fortress
- 3D SLAM: RTAB-Map (`src/SLAM/3D_SLAM/rtab_map_3d`)
- 2D SLAM: `src/SLAM/2D_SLAM/`
- Mapper: `src/Mapper/` (mapper, wall_detector)
- Nav2: `src/Planner/Nav2/nav2_smac_hybrid`
- Gazebo 시뮬레이터: `src/Gazebo/` (패키지명 `tm_gazebo`)
- Control: `src/Control/AMR-Motion-Control/`

## 시스템 환경

- 개발 PC: Intel Alder Lake-P iGPU + NVIDIA RTX 3080 Mobile (PRIME Optimus 랩톱)
- 64GB RAM, 20 CPU cores
- 배포 PC는 다른 GPU 구성일 수 있음 (순수 NVIDIA 데스크톱, 헤드리스 등)

## GPU 자동 선택 패턴 (배포 안전)

GUI launch 파일(Gazebo, RViz2, rqt 포함)은 **auto-detect + override** 패턴을 사용한다.
참조 구현: `src/Gazebo/launch/gazebo.launch.py` 의 `_detect_gpu_mode()` / `_setup_gpu_env()`.

- `gpu:=auto` (기본) — `xrandr --listproviders` 결과로 환경 자동 분기
  - Optimus(Intel+NVIDIA) → PRIME offload 환경변수 4종 자동 주입
  - 순수 NVIDIA 데스크톱 → 환경변수 불필요 (no-op)
  - DISPLAY 없음 → no-op
- `gpu:=nvidia_offload` — 강제 PRIME offload
- `gpu:=native` — 강제 비활성화 (디버깅, 헤드리스 SIL)

새 GUI launch 파일 작성 시 같은 헬퍼 함수와 `DeclareLaunchArgument('gpu', default='auto') + OpaqueFunction(_setup_gpu_env)` 패턴을 복사한다.

## 유틸리티

- ROS2 전체 종료: `~/Study/ros2_3dslam_ws/scripts/kill_all_ros2.sh`
