import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, NotSubstitution
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

from launch_ros.substitutions import FindPackageShare

# this is the function, that launch system will look for
def launch_setup(context):

    ### DATA INPUT ###
    vikings_bot_name = LaunchConfiguration("vikings_bot_name").perform(context)
    robot_file = LaunchConfiguration("robot_file").perform(context)
    use_sim = LaunchConfiguration("use_sim").perform(context)

    # robot_state_publisher_name = vikings_bot_name + "_robot_state_publisher"
    robot_state_publisher_name = "robot_state_publisher"
    ### DATA INPUT END ###

    package_description = 'vikings_bot_description'

    robot_file_extension = robot_file.split(".")[1]

    if robot_file_extension == "urdf":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path)
    elif robot_file_extension == "xacro":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path, mappings={"vikings_bot_name": vikings_bot_name,
                                                                    "use_sim": use_sim
                                                                  }
        )
    else:
        assert False, "Extension of robot is not supported: " + str(robot_file_extension)

    robot_xml = robot_desc.toxml()

    # Get rpbpt controller param file
    robot_controller_param_file = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            f"{vikings_bot_name}_diffbot_controllers.yaml",
        ]
    )

    
    control_node = Node(
        namespace=vikings_bot_name,
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_xml},
                    {robot_controller_param_file}],
        output="both",
        remappings=[
            # Only because of bug in diffdrive controller
            ("diffbot_base_controller/cmd_vel_unstamped", "cmd_vel"),
            ("diffbot_base_controller/odom", "odom_raw"),
        ],
        condition = IfCondition(
            NotSubstitution(LaunchConfiguration("use_sim"))
        ),
    )

    # Only because of bug in diffdrive controller
    odom_filter_node = Node(
        namespace=vikings_bot_name,
        package="diffdrive_roboteq_sbl",
        executable="odom_filter",
        output="both",
        condition = IfCondition(
            NotSubstitution(LaunchConfiguration("use_sim"))
        ),
    )


    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        name=robot_state_publisher_name,
        emulate_tty=True,
        namespace=vikings_bot_name,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim"),
            "robot_description": robot_xml,
            "frame_prefix": vikings_bot_name+"/",
        }],
        remappings=[],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        namespace=vikings_bot_name,
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        condition = IfCondition(
            NotSubstitution(LaunchConfiguration("use_sim"))
        ),
    )

    robot_controller_spawner = Node(
        namespace=vikings_bot_name,
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "controller_manager"],
        condition = IfCondition(
            NotSubstitution(LaunchConfiguration("use_sim"))
        ),
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition = IfCondition(
            NotSubstitution(LaunchConfiguration("use_sim"))
        ),
    )


    # #  RVIZ configuration file
    # rviz_file = "urdf_vis.rviz"
    # rviz_config_dir = os.path.join(get_package_share_directory(package_description), "rviz", rviz_file)

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="screen",
    #     parameters=[{
    #         "use_sim_time": True,
    #     }],
    #     arguments=["-d", rviz_config_dir]
    # )

    

    return [
        control_node,
        robot_state_publisher_node,
        odom_filter_node,
        joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner

    ]#rviz_node

def generate_launch_description(): 

    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
                    default_value="vikings_bot_1",
                    description="Namespace of robot - [vikings_bot_1 or vikings_bot_2]")
    robot_file_arg = DeclareLaunchArgument("robot_file", default_value="vikings_bot.xacro")
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="true")
    

    return LaunchDescription([
        vikings_bot_name_arg,
        robot_file_arg,
        use_sim_arg,
        OpaqueFunction(function = launch_setup)
        ])


