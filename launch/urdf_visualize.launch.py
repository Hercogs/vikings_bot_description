import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function, that launch system will look for
def generate_launch_description():

    ### DATA INPUT ###
    package_description = 'vikings_bot_description'
    robot_file = 'vikings_bot_geometric.urdf'
    #robot_file = 'vikings_bot.xacro'
    ### DATA INPUT END ###

    robot_file_extension = robot_file.split(".")[1]

    if robot_file_extension == "urdf":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path)
    elif robot_file_extension == "xacro":
        robot_desc_path = os.path.join(get_package_share_directory(package_description), robot_file_extension, robot_file)
        robot_desc = xacro.process_file(robot_desc_path, mappings={})
    else:
        assert False, "Extension of robot is not supported: " + str(robot_file_extension)

    robot_xml = robot_desc.toxml()

    # print(f'robot_desc: {robot_desc}')
    # print(f'robot_xml: {robot_xml}')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_xml,
            #"frame_prefix": "frame",
        }],
        remappings=[],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_node",
    )


    #  RVIZ configuration file
    rviz_file = "urdf_vis.rviz"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), "rviz", rviz_file)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
        arguments=["-d", rviz_config_dir]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node
        ]
    )


