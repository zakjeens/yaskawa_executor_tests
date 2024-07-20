from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the node (e.g., debug, info, warn, error, fatal)'
        ),
        DeclareLaunchArgument(
            'local_csv_path',
            default_value='/config/solutions_60p.csv',
            description='joint trajectory file in the package'
        ),

        # Define the Node action
        Node(
            package='yaskawa_executor_tests',
            executable='testTraj_client',
            name='testTraj_client',
            output='screen',
            parameters=[{
                'local_csv_path': LaunchConfiguration('local_csv_path'),
                'joint_names': ["group_1/joint_1", "group_1/joint_2", "group_1/joint_3", "group_1/joint_4", "group_1/joint_5", "group_1/joint_6"],  
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                '--ros-args',
                '--log-level', LaunchConfiguration('log_level')
            ]
        )
    ])
