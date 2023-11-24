import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

# this is the function, that launch system will look for
def launch_setup(context):

    ### DATA INPUT ###
    vikings_bot_name = LaunchConfiguration("vikings_bot_name").perform(context)
    robot_file = LaunchConfiguration("robot_file").perform(context)

    robot_state_publisher_name = vikings_bot_name + "_robot_state_publisher"
    ### DATA INPUT END ###

    package_description = 'vikings_bot_description'

    robot_file_extension = robot_file.split(".")[1]

    if robot_file_extension == "urdf":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path)
    elif robot_file_extension == "xacro":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path, mappings={"vikings_bot_name": vikings_bot_name})
    else:
        assert False, "Extension of robot is not supported: " + str(robot_file_extension)

    robot_xml = robot_desc.toxml()


    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        name=robot_state_publisher_name,
        emulate_tty=True,
        namespace=vikings_bot_name,
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_xml,
            "frame_prefix": vikings_bot_name+"/",
        }],
        remappings=[],
        output="screen",
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

    

    return [robot_state_publisher_node]#rviz_node

def generate_launch_description(): 

    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
                    default_value="vikings_bot",
                    description="Robot name to make it unique")
    robot_file_arg = DeclareLaunchArgument("robot_file", default_value="vikings_bot.xacro")
    

    return LaunchDescription([
        vikings_bot_name_arg,
        robot_file_arg,
        OpaqueFunction(function = launch_setup)
        ])


