from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('drone_bringup'), 'config', 'params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS 2 parameters file'),

        DeclareLaunchArgument(
            'tracking_enabled',
            default_value='false',
            description='Start with tracking enabled'),

        # Gazebo camera topic name — verify with: gz topic -l | grep camera
        DeclareLaunchArgument(
            'gz_camera_topic',
            default_value='/camera',
            description='Gazebo image topic to bridge into ROS 2'),

        Node(
            package='drone_controllers',
            executable='offboard_node',
            name='offboard_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        Node(
            package='drone_controllers',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # Bridge Gazebo camera → ROS 2 /camera/image_raw
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
            remappings=[('/camera', '/camera/image_raw')],
            output='screen',
        ),
    ])
