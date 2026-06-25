# Basic Robotics Experiment

WEGO Robotics의 Jetson Nano 기반 LIMO 모바일 로봇을 이용한 운전면허 기능시험 코스 보조 주행 로봇

<img src="image/field_environment_1.png" alt="경기장 환경 1" width="80%">

<img src="image/field_environment_2.png" alt="경기장 환경 2" width="80%">

## 프로젝트 목표

카메라, LiDAR, IMU, ArUco Marker를 이용해 LIMO 로봇이 기능시험 코스를 자율 주행하도록 구성

주요 기능:

- 노란색 차선 인식 기반 자율주행
- ArUco Marker 인식 후 정지, 좌회전, 우회전, 주차 동작 수행
- LiDAR 기반 장애물 및 차단기 감지 후 정지
- IMU 기반 방지턱 구간 감지 및 속도 제어
- 음성 안내 파일 재생 실험

## 사용 환경

| 항목 | 내용 |
| --- | --- |
| Robot | WEGO Robotics LIMO |
| Board | Jetson Nano |
| OS | Ubuntu 18.04 |
| ROS | ROS1 Melodic |
| Language | Python |
| Main sensors | RGB-D Camera, LiDAR, IMU |

## 노드 구성

| 파일 | 역할 |
| --- | --- |
| `lane_detect.py` | 카메라 이미지에서 노란색 차선 검출, 좌/우 차선 정보와 차선 기울기 값 퍼블리시 |
| `crosswalk_detect.py` | 흰색 횡단보도 패턴 검출, 횡단보도 거리 정보 퍼블리시 |
| `ar_marker.py` | ArUco Marker ID와 위치 기반 정지, 회전, 주차 명령 퍼블리시 |
| `lidar_stop.py` | LiDAR 전방 거리 기반 장애물 감지 여부와 감지 시간 퍼블리시 |
| `control.py` | 차선, 마커, LiDAR, IMU 정보를 통합해 최종 `/cmd_vel` 생성 |

## 주요 토픽 흐름

| 구분 | 토픽 |
| --- | --- |
| Camera input | `/camera/rgb/image_raw/compressed` |
| Lane output | `/limo/lane_left`, `/limo/lane_right`, `/limo/lane_connect`, `/limo/lane/accel`, `/limo/lane/gtan` |
| Crosswalk output | `/limo/crosswalk/distance` |
| Marker input/output | `/ar_pose_marker`, `/limo/marker/cmd_vel`, `/limo/marker/bool`, `/limo/marker/park` |
| LiDAR input/output | `/scan`, `/limo/lidar_warn`, `/limo/lidar_time`, `/limo/lidar/timer` |
| Control output | `/cmd_vel` |

## 기능 구현 내용

### 1. 전체 실행 흐름

`start.launch`는 LIMO 기본 bringup, 카메라 launch, 차선 인식, 횡단보도 인식, 주행 제어, LiDAR 정지 노드를 실행하는 시작점.

- `lane_detect.py`, `crosswalk_detect.py`, `control.py`, `lidar_stop.py`를 `limo_legend` 패키지 노드로 실행
- `control.py`에서 `/limo/lane_left` 데이터를 처음 수신하면 이미지 입력이 들어온 것으로 판단
- 이미지 수신 이후 `control.py`가 `marker.launch`를 실행해 ArUco Marker 관련 노드 시작
- `marker.launch`에서 `ar_track_alvar`와 `ar_marker.py` 실행

---

### 2. 차선 인식 (`lane_detect.py`)

`lane_detect.py`는 카메라 압축 이미지를 받아 노란색 차선 정보를 계산하는 노드.

- 입력: `/camera/rgb/image_raw/compressed`
- 이미지 처리: `CvBridge`로 OpenCV 이미지 변환 후 하단 영역 `[420:480, :]` 사용
- 색상 처리: BGR 이미지를 HLS로 변환하고 노란색 임계값으로 마스크 생성
- 좌우 분리: 하단 이미지를 `0:320`, `320:640` 영역으로 나누어 왼쪽 차선과 오른쪽 차선 처리
- 거리 계산: 각 마스크의 `cv2.moments` 무게중심 x좌표 계산, 검출 실패 시 `-1` 반환
- 곡선/겹침 판단: 왼쪽 ROI의 오른쪽 끝 열과 오른쪽 ROI의 왼쪽 끝 열에 차선 픽셀이 동시에 있으면 `/limo/lane_connect`에 `True` 퍼블리시
- 가속 판단: 두 ROI의 상단 행에서 차선이 동시에 검출되면 `/limo/lane/accel`에 `True` 퍼블리시
- 기울기 계산: 전체 노란색 마스크에서 픽셀 위치 기반 평균 기울기 값 `gtan` 계산

출력 토픽:

- `/limo/lane_left`: 왼쪽 차선 무게중심 x좌표
- `/limo/lane_right`: 오른쪽 차선 무게중심 x좌표
- `/limo/lane_connect`: 왼쪽 차선이 오른쪽 ROI까지 침범했는지 여부
- `/limo/lane/accel`: 두 차선이 안정적으로 검출되는 직진 구간 여부
- `/limo/lane/gtan`: 차선 평균 기울기 값

---

### 3. 횡단보도 인식 (`crosswalk_detect.py`)

`crosswalk_detect.py`는 카메라 이미지에서 흰색 횡단보도 패턴을 검출하는 노드.

- 입력: `/camera/rgb/image_raw/compressed`
- 이미지 처리: `CvBridge`로 OpenCV 이미지 변환 후 하단 중앙 영역 `[420:480, 170:530]` 사용
- 색상 처리: HLS 색공간에서 흰색 임계값으로 마스크 생성
- 에지 처리: Canny edge 검출 후 HoughLinesP로 선분 검출
- 횡단보도 판단: 검출된 선분 개수가 `CROSS_WALK_DETECT_TH` 이상이면 횡단보도 인식 상태로 판단
- 거리 계산: 횡단보도 인식 시 흰색 마스크의 무게중심 y좌표를 거리 값으로 사용, 미검출 시 `-1` 퍼블리시

출력 토픽:

- `/limo/crosswalk/distance`: 횡단보도 검출 시 y좌표 기반 거리 값, 미검출 시 `-1`

---

### 4. ArUco Marker 처리 (`ar_marker.py`)

`ar_marker.py`는 ArUco Marker ID, 마커까지의 거리, 횡단보도 인식 결과, 차선 기울기 값(`gtan`)을 함께 사용해 마커 동작 명령을 만드는 노드.

입력 토픽:

- `/ar_pose_marker`: `ar_track_alvar`가 퍼블리시하는 마커 위치와 ID
- `/limo/crosswalk/distance`: 횡단보도 인식 여부 판단용 거리 값
- `/limo/lane/gtan`: 차선 기울기 기반 회전 기준 값

Marker ID별 기본 동작:

| ID | 동작 |
| --- | --- |
| 0 | 정지 |
| 1 | 우회전 |
| 2 | 좌회전 |
| 3 | T자 주차 |

처리 방식:

- 마커의 x, y, z 위치값으로 로봇과 마커 사이 거리 계산
- ID 0은 정지 동작 후보로 처리
- ID 1은 `gtan > -0.5` 조건 또는 주차 이후 횡단보도 인식 조건에 따라 우회전 동작 선택
- ID 2는 `gtan < 0.5` 조건 또는 주차 이후 복귀 조건에 따라 좌회전 동작 선택
- ID 3은 주차 동작 후보로 처리
- 각 동작은 `rospy.get_time()`으로 계산한 경과 시간에 따라 선속도와 각속도를 단계적으로 변경
- 주차 동작과 주차 이후 복귀 동작 일부는 `gtan` 값을 이용해 차선과 평행해지는 지점까지 회전
- 동작 중 한 번만 mp3 파일을 재생하도록 `audio` 플래그 사용

출력 토픽:

- `/limo/marker/cmd_vel`: 마커 동작용 `Twist` 명령
- `/limo/marker/bool`: 마커 동작이 주행 제어를 override해야 하는지 여부
- `/limo/marker/park`: 주차 마커 동작 중인지 여부

---

### 5. LiDAR 장애물 처리 (`lidar_stop.py`)

`lidar_stop.py`는 전방 좁은 각도 범위의 LiDAR 거리값으로 장애물 여부를 판단하는 노드.

- 입력: `/scan`
- 검사 각도: -10도부터 10도까지 전방 영역
- 검사 거리: 0.3m 이내
- 판단 기준: 조건을 만족하는 LiDAR 포인트가 5개 이상이면 장애물 상태
- 장애물 감지 시 `/limo/lidar_warn`에 `Warning` 퍼블리시
- 장애물 미감지 시 `/limo/lidar_warn`에 `Safe` 퍼블리시하고 현재 시간보다 5초 뒤의 값을 `/limo/lidar/timer`에 퍼블리시

출력 토픽:

- `/limo/lidar_warn`: `Warning` 또는 `Safe`
- `/limo/lidar/timer`: 장애물 지속 감지를 판단하기 위한 기준 시간

---

### 6. 최종 주행 제어 (`control.py`)

`control.py`는 다른 노드의 결과를 모아 최종 `/cmd_vel`을 생성하는 중심 노드.

입력 토픽:

- `limo_status`: LIMO 주행 모드 확인
- `/imu`: 방지턱 통과 판단용 IMU 각속도
- `/limo/lane_left`, `/limo/lane_right`: 좌우 차선 위치
- `/limo/lane_connect`: 차선 겹침 여부
- `/limo/lane/accel`: 가속 가능 구간 여부
- `/limo/marker/cmd_vel`, `/limo/marker/bool`, `/limo/marker/park`: 마커 동작 명령과 상태
- `/limo/lidar_warn`, `/limo/lidar/timer`: LiDAR 장애물 상태

제어 계산 순서:

- 기본 선속도는 `BASE_SPEED = 0.3`
- 좌우 차선 위치와 기준값 `REF_X`, `REF_X2`의 차이를 이용해 조향값 계산
- `/limo/lane_connect`가 `True`이면 오른쪽 차선 계산값을 제외하고 왼쪽 차선 기준으로 조향
- `/limo/marker/bool`이 `True`이면 차선 주행 명령 대신 `/limo/marker/cmd_vel` 명령 사용
- 차선 겹침이 없고 `/limo/lane/accel`이 `True`이며 조향값이 작고 주차 중이 아니면 선속도 1.3배 적용
- IMU y축 각속도 적분값의 절댓값이 0.05보다 크면 방지턱 구간으로 판단
- LiDAR timer가 현재 시간보다 작으면 장시간 장애물 감지 상태로 보고 제자리 회전 명령 적용
- `/limo/lidar_warn`가 `Warning`이면 정지 명령 적용
- 방지턱 구간이면 선속도를 절반으로 줄이고 각속도를 0으로 설정
- LIMO가 differential mode일 때만 최종 `Twist`를 `/cmd_vel`로 퍼블리시

출력 토픽:

- `/cmd_vel`: 최종 주행 명령

---

### 7. Voice Module

음성 안내는 `ar_marker.py` 내부에서 마커 동작과 함께 처리되는 보조 기능.

- 음성 파일 위치: `src/limo_legend/audio/`
- 재생 방식: `pygame.mixer.music.load()`와 `pygame.mixer.music.play()` 사용
- 재생 조건: 정지, 우회전, 좌회전, 주차 동작 진입 시 해당 mp3 파일 재생
- 중복 방지: 하나의 마커 동작 중 `audio` 플래그로 반복 재생 방지
- 실제 시연 반영: 주행 중 지연 문제로 실제 시연에서는 제외
