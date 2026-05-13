from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # ===== Topics =====
    points_topic_arg = DeclareLaunchArgument(
        "points_topic",
        default_value="/ac1/pointcloud",
        description="LiDAR point cloud topic",
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/ac1/imu",
        description="IMU topic",
    )

    # ===== Frames =====
    odom_child_frame_id_arg = DeclareLaunchArgument(
        "odom_child_frame_id",
        default_value="ac1_lidar",
        description="Sensor / base frame to which the point cloud will be transformed",
    )
    robot_odom_frame_id_arg = DeclareLaunchArgument(
        "robot_odom_frame_id",
        default_value="odom",
        description="Robot odom frame (parent of odom_child_frame_id)",
    )

    # ===== Map =====
    globalmap_pcd_arg = DeclareLaunchArgument(
        "globalmap_pcd",
        default_value="/root/ros2_ws/src/q_gist_office_ac1.pcd",
        description="Path to the global map PCD file",
    )

    # ===== Lidar -> IMU offset (AC1 device) =====
    lidar_to_imu_x_arg = DeclareLaunchArgument(
        "lidar_to_imu_x", default_value="-0.0106"
    )
    lidar_to_imu_y_arg = DeclareLaunchArgument(
        "lidar_to_imu_y", default_value="-0.0099"
    )
    lidar_to_imu_z_arg = DeclareLaunchArgument("lidar_to_imu_z", default_value="0.0155")

    # ===== Optional flags =====
    use_imu_arg = DeclareLaunchArgument("use_imu", default_value="false")
    invert_imu_acc_arg = DeclareLaunchArgument("invert_imu_acc", default_value="false")
    invert_imu_gyro_arg = DeclareLaunchArgument(
        "invert_imu_gyro", default_value="false"
    )
    use_global_localization_arg = DeclareLaunchArgument(
        "use_global_localization", default_value="false"
    )
    enable_robot_odometry_prediction_arg = DeclareLaunchArgument(
        "enable_robot_odometry_prediction", default_value="false"
    )

    points_topic = LaunchConfiguration("points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_child_frame_id = LaunchConfiguration("odom_child_frame_id")
    robot_odom_frame_id = LaunchConfiguration("robot_odom_frame_id")
    use_imu = LaunchConfiguration("use_imu")
    invert_imu_acc = LaunchConfiguration("invert_imu_acc")
    invert_imu_gyro = LaunchConfiguration("invert_imu_gyro")
    use_global_localization = LaunchConfiguration("use_global_localization")
    enable_robot_odometry_prediction = LaunchConfiguration(
        "enable_robot_odometry_prediction"
    )

    # ===== Static TFs =====
    # odom -> ac1_lidar (identity). hdl_localization re-publishes map -> odom on top of this.
    odom_to_lidar_tf = Node(
        name="odom_to_lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "odom",
            "--child-frame-id",
            "ac1_lidar",
        ],
    )
    # ac1_lidar -> ac1_imu (extrinsic from AC1 driver)
    lidar_to_imu_tf = Node(
        name="lidar_to_imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            LaunchConfiguration("lidar_to_imu_x"),
            "--y",
            LaunchConfiguration("lidar_to_imu_y"),
            "--z",
            LaunchConfiguration("lidar_to_imu_z"),
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "ac1_lidar",
            "--child-frame-id",
            "ac1_imu",
        ],
    )

    container = ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::GlobalmapServerNodelet",
                name="GlobalmapServerNodelet",
                parameters=[
                    {"globalmap_pcd": LaunchConfiguration("globalmap_pcd")},
                    {"convert_utm_to_local": False},
                    {"downsample_resolution": 0.1},
                ],
            ),
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::HdlLocalizationNodelet",
                name="HdlLocalizationNodelet",
                remappings=[
                    ("/velodyne_points", points_topic),
                    ("/gpsimu_driver/imu_data", imu_topic),
                ],
                parameters=[
                    {"odom_child_frame_id": odom_child_frame_id},
                    {"use_imu": use_imu},
                    {"invert_acc": invert_imu_acc},
                    {"invert_gyro": invert_imu_gyro},
                    {"cool_time_duration": 2.0},
                    {
                        "enable_robot_odometry_prediction": enable_robot_odometry_prediction
                    },
                    {"robot_odom_frame_id": robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {"reg_method": "NDT_OMP"},
                    {"ndt_neighbor_search_method": "DIRECT7"},
                    {"ndt_neighbor_search_radius": 1.0},
                    {"ndt_resolution": 1.0},
                    {"downsample_resolution": 0.1},
                    {"specify_init_pose": True},
                    {"init_pos_x": 0.0},
                    {"init_pos_y": 0.0},
                    {"init_pos_z": 0.0},
                    {"init_ori_w": 1.0},
                    {"init_ori_x": 0.0},
                    {"init_ori_y": 0.0},
                    {"init_ori_z": 0.0},
                    {"use_global_localization": use_global_localization},
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            points_topic_arg,
            imu_topic_arg,
            odom_child_frame_id_arg,
            robot_odom_frame_id_arg,
            globalmap_pcd_arg,
            lidar_to_imu_x_arg,
            lidar_to_imu_y_arg,
            lidar_to_imu_z_arg,
            use_imu_arg,
            invert_imu_acc_arg,
            invert_imu_gyro_arg,
            use_global_localization_arg,
            enable_robot_odometry_prediction_arg,
            launch_ros.actions.SetParameter(name="use_sim_time", value=True),
            odom_to_lidar_tf,
            lidar_to_imu_tf,
            container,
        ]
    )
