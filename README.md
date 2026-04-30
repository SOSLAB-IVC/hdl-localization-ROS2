# hdl-localization-ROS2

3D LiDAR 기반 실시간 위치 추정(localization) 패키지의 ROS2 포팅판입니다. 미리 만들어둔 PCD 글로벌 맵 위에서 NDT(또는 NDT-OMP / NDT-CUDA) 스캔 매칭과 IMU UKF 융합을 통해 `map → odom → base` TF 체인을 산출합니다.

원본 ROS1 패키지([koide3/hdl_localization](https://github.com/koide3/hdl_localization))를 ROS2 Foxy 환경에 맞게 정리한 것이며, 로컬 워크스페이스(`ros2_ws`)에서 직접 빌드하여 사용합니다.

---

## 1. 패키지 구성

```
hdl-localization-ROS2/
├── hdl_localization/           # 메인 localization 노드 (NDT 스캔 매칭 + UKF)
│   ├── apps/                   # GlobalmapServerNodelet, HdlLocalizationNodelet
│   ├── launch/hdl_localization.py
│   ├── data/                   # 샘플 PCD 글로벌 맵
│   ├── rviz/                   # RViz 설정
│   └── sample-bag/             # 테스트용 rosbag2
├── hdl_global_localization/    # BBS / FPFH-RANSAC 기반 글로벌 위치 추정
├── fast_gicp/                  # GICP / VGICP 등록기 (CUDA 옵션 가능)
└── ndt_omp/                    # OpenMP 가속 NDT
```

---

## 2. 요구 사항

| 항목       | 버전 / 비고                                              |
| ---------- | -------------------------------------------------------- |
| OS         | Ubuntu 20.04                                             |
| ROS 2      | Foxy Fitzroy (검증) / Humble·Jazzy는 일부 빌드 수정 필요 |
| PCL        | 1.10 이상                                                |
| Eigen      | 3.3 이상                                                 |
| GPU (선택) | NDT-CUDA / fast_gicp CUDA 사용 시 NVIDIA GPU             |

세부 의존 패키지는 [3. 설치](#3-설치) 섹션에서 한 번에 설치합니다.

---

## 3. 설치

`~/ros2_ws` 워크스페이스 기준으로 진행합니다.

### 3-1. 워크스페이스에 소스 클론

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/SOSLAB-IVC/hdl-localization-ROS2
```

### 3-2. 시스템 라이브러리 설치 (PCL / Eigen / GLOG 등)

```bash
sudo apt update
sudo apt install -y \
    cmake build-essential \
    libatlas-base-dev libeigen3-dev libpcl-dev \
    libgoogle-glog-dev libsuitesparse-dev libglew-dev libpcap-dev \
    libomp-dev
```

### 3-3. 필수 ROS 2 패키지 설치

아래 명령은 `$ROS_DISTRO`를 그대로 사용하므로 Foxy / Humble / Jazzy 등 어느 배포판에서도 동작합니다. 셸을 새로 연 직후라면 ROS 2를 먼저 source 하거나 변수를 직접 지정하세요.

```bash
source /opt/ros/foxy/setup.bash       # 또는 humble / jazzy / rolling 등
# 또는 명시적으로 지정
export ROS_DISTRO=foxy
```

```bash
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    python3-colcon-common-extensions
```

> **rosdep 대안**: 처음 사용한다면 `sudo rosdep init && rosdep update`를 먼저 실행한 뒤 아래 한 줄로 `package.xml`에 선언된 의존성을 자동 설치할 수 있습니다.
>
> ```bash
> cd ~/ros2_ws
> rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
> ```

### 3-4. 빌드 및 환경 등록

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

> 빌드 순서 의존성: `ndt_omp` → `fast_gicp` → `hdl_global_localization` → `hdl_localization`. `colcon`이 자동으로 처리하지만, 한 패키지만 다시 빌드할 때는 `--packages-select` 옵션을 사용하세요.

---

## 4. 셋팅 (Launch 파라미터)

런치 파일: [hdl_localization/launch/hdl_localization.py](hdl_localization/launch/hdl_localization.py)

| 인자                                 | 기본값                                             | 설명                                   |
| ------------------------------------ | -------------------------------------------------- | -------------------------------------- |
| `points_topic`                       | `/ac1/pointcloud`                                  | 입력 LiDAR 포인트클라우드 토픽         |
| `imu_topic`                          | `/ac1/imu`                                         | IMU 토픽 (`use_imu:=false`이면 무시)   |
| `odom_child_frame_id`                | `ac1_lidar`                                        | 포인트클라우드의 센서 프레임           |
| `robot_odom_frame_id`                | `odom`                                             | 로봇 odom 프레임 (부모)                |
| `globalmap_pcd`                      | `/root/ros2_ws/src/map_publisher/maps/raw_map.pcd` | 글로벌 맵 PCD 경로                     |
| `lidar_to_imu_x/y/z`                 | `-0.0106 / -0.0099 / 0.0155`                       | LiDAR → IMU 변환 (AC1 디바이스 기본값) |
| `use_imu`                            | `true`                                             | IMU 사용 여부                          |
| `invert_imu_acc` / `invert_imu_gyro` | `false`                                            | IMU 축 반전                            |
| `use_global_localization`            | `false`                                            | 초기 위치 추정에 BBS/FPFH-RANSAC 사용  |
| `enable_robot_odometry_prediction`   | `false`                                            | 휠 오도메트리로 prediction step 보정   |

런치 파일 내부에서 추가로 설정되는 주요 노드 파라미터:

- `reg_method`: `NDT_OMP` (기본) / `NDT_CUDA_P2D` / `NDT_CUDA_D2D`
- `ndt_neighbor_search_method`: `DIRECT7`
- `ndt_resolution`: `1.0` m
- `downsample_resolution`: `0.05` m
- `specify_init_pose`: `true`, 초기 자세 `init_pos_*`, `init_ori_*`로 지정
- `cool_time_duration`: `2.0` s — 부팅 직후 매칭 비활성화 구간

> **TF 체인**: 런치 파일이 `odom → ac1_lidar` (identity)와 `ac1_lidar → ac1_imu` (extrinsic) 두 개의 static TF를 발행합니다. localization 노드는 그 위에 `map → odom`을 발행하므로, 다른 로봇/프레임 이름을 쓰는 경우 두 static TF의 frame-id 인자를 함께 바꿔야 합니다.

---

## 5. 사용법

### 5-1. 실행 (3-터미널 구성)

세 개의 터미널을 사용합니다(모두 `~/ros2_ws` 기준).

```bash
# 터미널 1 — localization 실행
source ~/ros2_ws/install/setup.bash
ros2 launch hdl_localization hdl_localization.py
```

```bash
# 터미널 2 — RViz 가시화
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/hdl-localization-ROS2/hdl_localization/rviz/hdl_localization_ros2.rviz
```

```bash
# 터미널 3 — 샘플 bag 재생 (또는 실제 센서 드라이버)
source ~/ros2_ws/install/setup.bash
ros2 bag play ~/ros2_ws/src/hdl-localization-ROS2/hdl_localization/sample-bag/subset/
```

> 샘플 bag을 재생할 때는 런치 파일의 `use_sim_time:=true`(기본)이 적용되어 있어야 시간 동기화가 맞습니다. 실제 하드웨어 사용 시에는 `use_sim_time:=false`로 오버라이드하세요.

### 5-2. 초기 위치 지정

- 정적인 초기 자세를 알고 있다면 런치 파일의 `init_pos_*`, `init_ori_*` 값을 수정합니다.
- 초기 자세를 모르는 경우 `use_global_localization:=true`로 실행하고, RViz의 `2D Pose Estimate`로 대략적 위치를 찍어 BBS/FPFH-RANSAC 글로벌 추정을 트리거합니다.

### 5-3. 주요 입출력

**Subscribe**

- `/velodyne_points` (`sensor_msgs/PointCloud2`) — `points_topic`으로 리매핑됨
- `/gpsimu_driver/imu_data` (`sensor_msgs/Imu`) — `imu_topic`으로 리매핑됨

**Publish**

- `/odom` (`nav_msgs/Odometry`) — 추정된 로봇 자세
- `/aligned_points` (`sensor_msgs/PointCloud2`) — 정합된 스캔
- `/status` (`hdl_localization/ScanMatchingStatus`) — 매칭 품질
- TF: `map → odom`
