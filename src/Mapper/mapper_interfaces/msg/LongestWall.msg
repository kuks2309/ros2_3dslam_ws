# 라이다 스캔에서 검출된 최장 벽 정보
# Publisher: wall_detector_node (Hough 등 알고리즘)
# Subscriber: wall_aligner_node (정렬 제어)

# 검출 방법 enum
uint8 METHOD_HOUGH  = 1
uint8 METHOD_RANSAC = 2

# ── 헤더 ──────────────────────────────────────────────
# stamp: 검출 시점 (stale 필터 용)
# frame_id: 검출 좌표계 (예: lidar_link)
std_msgs/Header header

# ── 벽 기하 정보 (모두 robot/lidar frame 기준) ─────
# angle_rad: 벽 방향 [-π/2, π/2) (ROS 관례, deg 아님)
float64 angle_rad

# length_m: 검출된 선분 길이 (m)
float64 length_m

# distance_m: 로봇으로부터 벽까지 수직 거리 (m, 정렬 후 접근에 사용)
float64 distance_m

# ── 검출 신뢰도 ───────────────────────────────────
# inlier_count: 선분에 일치하는 라이다 점 개수
int32 inlier_count

# inlier_ratio: 선분 위의 점 비율 [0.0, 1.0]
float64 inlier_ratio

# confidence: 동일 frame 내 검출된 벽 중 상대 순위 [0.0, 1.0]
# 1.0 = 가장 많은 inlier를 가진 벽. 절대 신뢰도 아님 — 시계열 비교 무의미.
float64 confidence

# ── 메타 ──────────────────────────────────────────
# detection_method: METHOD_HOUGH(1) 사용 중. METHOD_RANSAC(2)은 향후 폴백 예약.
uint8 detection_method

# ── 선분 양 끝점 (시각화·디버깅) ─────────────────
geometry_msgs/Point start
geometry_msgs/Point end
