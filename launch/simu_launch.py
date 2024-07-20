from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument
import xacro
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#========================================== THE ROBOT ===========================================

robot_model_config_pkg = "robot_description_moveit_config"
config_dir_name = "config"
robot_name = "robot_description"

#===================================================== functions  =====================================================
def evaluate_substitution(context, substitution):
    return substitution.perform(context)
    
# ===============================================================================================================================================================
def generate_launch_description():

    # ===================================================== Paths and Dictionaries =====================================================
    context = LaunchContext()

    robot_description_path = PathJoinSubstitution(
           [FindPackageShare(robot_model_config_pkg), config_dir_name, f"{robot_name}.urdf.xacro"])

    robot_description_config = xacro.process_file(evaluate_substitution(context, robot_description_path))

    robot_description = {"robot_description": robot_description_config.toxml()}

     # ================================
    rviz_config_file_path = PathJoinSubstitution(
           [FindPackageShare("yaskawa_executor_tests"), config_dir_name, "dummy_rviz.rviz"])

    rviz_config_file = evaluate_substitution(context, rviz_config_file_path)


    # ===================================================== Robot State Publisher =====================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="both",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )

     
    # ===================================================== RVIZ =====================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )
    

    # ===================================================== Joint state publisher =====================================================

    # Start joint_state_publisher
    joint_state_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'rate': LaunchConfiguration('rate', default = '50')},
                {'source_list': LaunchConfiguration('source_list', default = '[/dummy_joint_states]')}
            ]
        )

    # ===================================================== Launch Description =====================================================

    return LaunchDescription(
        [
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
                'response_delay',
                default_value='0.1',
                description='sleep time before accepting the new point'
            ),
            DeclareLaunchArgument(
                'busy_count',
                default_value='2',
                description='amount of times the server will reply with BUSY before accepting'
            ),
            # Define the dummy node
            Node(
                package='yaskawa_executor_tests',
                executable='dummy_qtp_service',
                name='dummy_qtp_service',
                output='screen',
                parameters=[{
                    'response_delay': LaunchConfiguration('response_delay'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'busy_count': LaunchConfiguration('busy_count')
                }],
                arguments=[
                    '--ros-args',
                    '--log-level', LaunchConfiguration('log_level')
                ]
            ),
            robot_state_publisher,
            rviz_node,
            joint_state_node,
        ]
    )

