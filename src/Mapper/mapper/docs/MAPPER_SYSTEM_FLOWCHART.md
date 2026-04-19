# Mapper System 전체 Flowchart

## 1. 시스템 구성 (노드·인터페이스)

```mermaid
flowchart LR
    UI[mapper_ui<br/>PyQt5 GUI] -->|CMD_*| CMD[/mapper/command<br/>service/]
    CMD --> ORCH[mapper_orchestrator<br/>상태 머신]
    ORCH -->|state| STATUS[/mapper/status<br/>topic/]
    STATUS --> UI

    ORCH -->|WallAlign| WA[wall_aligner<br/>RANSAC 벽 탐지]
    ORCH -->|MapAlignmentCheck| MAC[map_alignment_checker<br/>지도 축 정합]
    ORCH -->|ExploreUnknown| EXP[exploration_planner<br/>Nav2 기반 탐사]
    ORCH -->|SlamControl| SLAM[slam_manager_2d/3d<br/>SLAM 기동·저장]

    WA -->|Spin Action| NAV2[Nav2 Behaviors<br/>spin]
    EXP -->|NavigateToPose| NAV2
    LIDAR[/scan LaserScan/] --> WA
    LIDAR --> MAC
    LIDAR --> SLAM
```

## 2. 상태 머신 (State Machine)

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> ALIGNING: CMD_START_MAPPING
    ALIGNING --> STARTING_SLAM: 정렬 완료
    ALIGNING --> ERROR: max_retries 초과
    STARTING_SLAM --> VERIFYING_MAP: SLAM 시작 성공
    STARTING_SLAM --> ERROR: SLAM 시작 실패
    VERIFYING_MAP --> MAPPING_MANUAL: aligned AND drive_mode=MANUAL
    VERIFYING_MAP --> MAPPING_AUTO: aligned AND drive_mode=AUTO
    VERIFYING_MAP --> ALIGNING: misaligned (재정렬)
    VERIFYING_MAP --> ERROR: max_realigns 초과

    MAPPING_MANUAL --> EXPLORING_UNKNOWN: CMD_EXPLORE
    MAPPING_MANUAL --> LOOP_CLOSING: coverage ≥ 95%
    MAPPING_AUTO --> EXPLORING_UNKNOWN: run_mapping_auto
    EXPLORING_UNKNOWN --> LOOP_CLOSING: 탐사 완료
    EXPLORING_UNKNOWN --> MAPPING_MANUAL: 탐사 실패

    MAPPING_MANUAL --> PAUSED: CMD_PAUSE
    MAPPING_AUTO --> PAUSED: CMD_PAUSE
    EXPLORING_UNKNOWN --> PAUSED: CMD_PAUSE
    PAUSED --> MAPPING_MANUAL: CMD_RESUME (이전=MANUAL)
    PAUSED --> MAPPING_AUTO: CMD_RESUME (이전=AUTO)
    PAUSED --> EXPLORING_UNKNOWN: CMD_RESUME (이전=EXPLORE)

    LOOP_CLOSING --> COMPLETED: loop closure OR timeout

    IDLE --> IDLE: CMD_STOP
    ALIGNING --> IDLE: CMD_STOP
    STARTING_SLAM --> IDLE: CMD_STOP
    VERIFYING_MAP --> IDLE: CMD_STOP
    MAPPING_MANUAL --> IDLE: CMD_STOP
    MAPPING_AUTO --> IDLE: CMD_STOP
    EXPLORING_UNKNOWN --> IDLE: CMD_STOP
    LOOP_CLOSING --> IDLE: CMD_STOP
    PAUSED --> IDLE: CMD_STOP
    ERROR --> IDLE: CMD_STOP

    COMPLETED --> [*]
    ERROR --> [*]
```

## 3. 전체 매핑 워크플로우

```mermaid
flowchart TD
    START([CMD_START_MAPPING<br/>slam_mode, drive_mode]) --> IDLE{state = IDLE?}
    IDLE -- No --> REJ([reject: Not in IDLE])
    IDLE -- Yes --> ALIGN[상태 ALIGNING<br/>run_aligning]

    ALIGN --> WALIGN[WallAlign Action<br/>RANSAC 벽 탐지<br/>Spin 회전 보정]
    WALIGN --> AOK{정렬 성공?}
    AOK -- No --> ARETRY{retry &lt; max?}
    ARETRY -- Yes --> ALIGN
    ARETRY -- No --> ERR([상태 ERROR])
    AOK -- Yes --> SSLAM[상태 STARTING_SLAM<br/>run_starting_slam]

    SSLAM --> SLAMSVC[SlamControl<br/>CMD_START_2D / 3D]
    SLAMSVC --> SOK{SLAM 시작 성공?}
    SOK -- No --> ERR
    SOK -- Yes --> TF[TF map→base_link 조회<br/>start_x, start_y 기록]
    TF --> VMAP[상태 VERIFYING_MAP<br/>run_verifying_map]

    VMAP --> MAC[MapAlignmentCheck Action]
    MAC --> MOK{축 정합 OK?}
    MOK -- No --> RETRY{realign &lt; max?}
    RETRY -- Yes --> ALIGN
    RETRY -- No --> ERR
    MOK -- Yes --> MODE{drive_mode?}

    MODE -- MANUAL --> MMANUAL[상태 MAPPING_MANUAL<br/>run_mapping_manual]
    MODE -- AUTO --> MAUTO[상태 MAPPING_AUTO<br/>run_mapping_auto]

    MMANUAL --> MLOOP{coverage &ge; 95%<br/>OR CMD_EXPLORE?}
    MLOOP -- coverage --> LC
    MLOOP -- EXPLORE --> EXPU
    MLOOP -- 대기 --> MMANUAL

    MAUTO --> EXPU[상태 EXPLORING_UNKNOWN<br/>run_exploring]
    EXPU --> EGOAL[ExploreUnknown Action<br/>Nav2 기반 탐사]
    EGOAL --> EOK{탐사 성공?}
    EOK -- Yes --> LC[상태 LOOP_CLOSING<br/>run_loop_closing]
    EOK -- No --> MMANUAL

    LC --> LCWAIT[루프 클로저 대기<br/>timeout=30s]
    LCWAIT --> DONE([상태 COMPLETED])
```

## 4. 벽 정렬(WallAlign) 상세 플로우

```mermaid
flowchart TD
    A([WallAlign Goal<br/>tolerance_deg]) --> SCAN[/scan 최신 수신]
    SCAN --> RANSAC[RANSAC 최장 벽 탐지<br/>iterations=200<br/>inlier_dist=0.05m]
    RANSAC --> WALL[wall_angle_deg]
    WALL --> TF[TF map→base_link 조회<br/>robot_yaw_deg]
    TF --> ERR[error = yaw - wall_angle<br/>정규화 -90~90°]
    ERR --> CHECK{abs error ≤ tolerance?}
    CHECK -- Yes --> SUCCESS([status=0<br/>aligned_heading 반환])
    CHECK -- No --> SPIN[Spin Action 전송<br/>target = yaw - error]
    SPIN --> RETRY{attempt &lt; max?}
    RETRY -- Yes --> SCAN
    RETRY -- No --> FAIL([status=-1 ABORT])
```

## 5. 맵 정합 검사(MapAlignmentCheck) 플로우

```mermaid
flowchart TD
    A([MapAlignmentCheck Goal<br/>tolerance_deg]) --> MAP[/map OccupancyGrid 수신]
    MAP --> EDGE[Canny 엣지 검출]
    EDGE --> HOUGH[Hough 변환<br/>주방향 추출]
    HOUGH --> ANGLE[주 축 각도 계산]
    ANGLE --> CHECK{abs angle &le; tolerance?}
    CHECK -- Yes --> OK([is_aligned=true])
    CHECK -- No --> NOT([is_aligned=false<br/>재정렬 필요])
```

## 6. UI 상호작용 플로우

```mermaid
flowchart TD
    USER([사용자 버튼 클릭]) --> UI[PyQt5 GUI]
    UI -->|send_command| BRIDGE[RosBridge<br/>mapper/command client]
    BRIDGE -->|call_async| ORCH[mapper_orchestrator]
    ORCH -->|response| CB[_on_command_response]
    CB -->|성공/실패 로그| LOG[_on_log]

    ORCH -->|publish status| TOPIC[/mapper/status/]
    TOPIC -->|status_sub 콜백| BRIDGE2[RosBridge._on_status]
    BRIDGE2 -->|pyqtSignal| UI_UPDATE[UI 업데이트<br/>상태 라벨·버튼 활성화<br/>coverage 바·헤딩 오차]

    UI -->|수동 주행 버튼 pressed| TWIST[publish Twist to /cmd_vel]
    UI -->|released| STOP[publish 0,0]
```

## 7. 핵심 파라미터 요약

| 파라미터 | 기본값 | 출처 | 역할 |
|---------|--------|------|------|
| `max_align_retries` | 3 | orchestrator | WallAlign 재시도 |
| `map_stabilize_wait_sec` | 3.0 | orchestrator | SLAM 시작 후 지도 안정화 대기 |
| `loop_closure_timeout_sec` | 30.0 | orchestrator | 루프 클로저 대기 타임아웃 |
| `min_coverage_to_stop` | 0.95 | orchestrator | 매핑 완료 커버리지 임계 |
| `tolerance_deg` | 0.2 | wall_aligner | 정렬 각도 오차 허용 |
| `ransac_iterations` | 200 | wall_aligner | RANSAC 반복 |
| `inlier_dist_m` | 0.05 | wall_aligner | Inlier 거리 임계 |
| `min_inliers` | 10 | wall_aligner | 최소 inlier 개수 |
| `spin_speed_deg_s` | 40.0 | wall_aligner | 회전 속도 |

## 8. 상태-명령 호환성 매트릭스

| 상태 | START | PAUSE | RESUME | STOP | SAVE_MAP | EXPLORE |
|------|:-----:|:-----:|:------:|:----:|:--------:|:-------:|
| IDLE | ✅ | ❌ | ❌ | ✅ | ✅* | ❌ |
| ALIGNING | ❌ | ❌ | ❌ | ✅ | ✅* | ❌ |
| STARTING_SLAM | ❌ | ❌ | ❌ | ✅ | ✅* | ❌ |
| VERIFYING_MAP | ❌ | ❌ | ❌ | ✅ | ✅* | ❌ |
| MAPPING_MANUAL | ❌ | ✅ | ❌ | ✅ | ✅ | ✅ |
| MAPPING_AUTO | ❌ | ✅ | ❌ | ✅ | ✅ | ❌ |
| EXPLORING_UNKNOWN | ❌ | ✅ | ❌ | ✅ | ✅ | ❌ |
| LOOP_CLOSING | ❌ | ❌ | ❌ | ✅ | ✅ | ❌ |
| PAUSED | ❌ | ❌ | ✅ | ✅ | ✅ | ❌ |
| COMPLETED | ❌ | ❌ | ❌ | ❌ | ✅ | ❌ |
| ERROR | ❌ | ❌ | ❌ | ✅ | ❌ | ❌ |

✅* = SLAM 서비스 ready일 때만 성공

## 9. 퍼블리시/서브스크라이브 토픽

| 방향 | 토픽 | 타입 | QoS | 발행/구독 노드 |
|-----|------|------|-----|---------------|
| 출력 | `mapper/status` | MapperStatus | RELIABLE | orchestrator → UI |
| 입력 | `scan` | LaserScan | BEST_EFFORT | wall_aligner, map_alignment_checker |
| 입력 | `map` | OccupancyGrid | TRANSIENT_LOCAL | orchestrator, map_alignment_checker |
| 출력 | `cmd_vel` | Twist | default | mapper_ui (수동 주행) |

## 10. 서비스/액션

| 이름 | 타입 | 서버 | 클라이언트 |
|------|------|------|-----------|
| `mapper/command` | Service MapperCommand | orchestrator | UI |
| `wall_align` | Action WallAlign | wall_aligner | orchestrator |
| `map_alignment_check` | Action MapAlignmentCheck | map_alignment_checker | orchestrator |
| `explore_unknown` | Action ExploreUnknown | exploration_planner | orchestrator |
| `spin` | Action Spin (Nav2) | Nav2 Behaviors | wall_aligner |
| `slam_manager_2d/slam_control` | Service SlamControl | slam_manager_2d | orchestrator |
