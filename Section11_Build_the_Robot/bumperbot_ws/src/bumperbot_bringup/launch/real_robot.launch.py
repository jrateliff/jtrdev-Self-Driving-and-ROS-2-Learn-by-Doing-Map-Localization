import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Firmware/Hardware interface
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )

    # RPLIDAR A1 driver
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen"
    )

    # Controller nodes
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items()
    )

    # Joystick control
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # Intel RealSense D435i (IMU + Depth)
    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        output="screen",
        parameters=[{
            "enable_gyro": True,
            "enable_accel": True,
            "enable_depth": True,
            "enable_color": True,
            "color_width": 640,
            "color_height": 480,
            "color_fps": 30,
            "unite_imu_method": 2,  # Correct integer value
            "publish_odom_tf": False,
            "base_frame_id": "d435i_link",
            "imu_frame_id": "d435i_imu_link",
            "depth_frame_id": "d435i_depth_link",
            "gyro_frame_id": "d435i_imu_link",
            "accel_frame_id": "d435i_imu_link",
            "topic_odom_in": "",
        }],
        remappings=[
            ("/camera/realsense2_camera/imu", "/imu_ekf")
        ]
    )

    # Safety stop node
    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen"
    )

    # Localization if not using SLAM
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        condition=UnlessCondition(use_slam)
    )

    # SLAM toolbox if enabled
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        camera_node,
        safety_stop,
        localization,
        slam
    ])
