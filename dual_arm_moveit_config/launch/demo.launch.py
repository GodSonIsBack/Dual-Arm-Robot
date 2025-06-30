
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_arms",package_name="dual_arm_moveit_config")
        .robot_description(file_path="config/dual_arms.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arms.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl","chomp","pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_moveit_config"),
        "config","moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    static_tf_node_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_arm_publisher",
        arguments=["0.0","-1.5","0.0","0.0","0.0","0.0","world","left_panda_link0"]
    )
    static_tf_node_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_arm_publisher",
        arguments=["0.0","1.5","0.0","0.0","0.0","0.0","world","right_panda_link0"]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
    )
    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_hand_controller_spawner,
            right_arm_controller_spawner,
            right_hand_controller_spawner,
            static_tf_node_left,
            static_tf_node_right
            
        ]
    )
    
