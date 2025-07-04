import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():
    
    moveit_configs=(
        MoveItConfigsBuilder("iiwa_dual_arm",package_name="dual_arm_moveit_config_iiwa")
        .robot_description(file_path="config/dual_arm_iiwa.urdf.xacro")
        .robot_description_semantic(file_path="config/iiwa_dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True,
                                publish_planning_scene=True,publish_geometry_updates=True,
                                publish_state_updates=True,publish_transforms_updates=True)
        .planning_pipelines(pipelines=["ompl","chomp"])
        .to_moveit_configs()
        )
    move_group_configuration = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            # Note: Wrapping the following values is necessary so that the parameter value can be the empty string

            # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates":True,
            "monitor_dynamics": False,
        }
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_configs.to_dict(),move_group_configuration],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    controller_path =os.path.join(get_package_share_directory("dual_arm_moveit_config_iiwa"),"config","lbr_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_configs.robot_description,controller_path],
        remappings=[("~/robot_description", "robot_description")]
    )
    
    left_static_tf_node = Node(
        package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',     # x, y, z translation
                '0', '0', '0',     # roll, pitch, yaw in radians
                'world',          # parent frame
                'left_link_0'         # child frame
            ],
            output='screen'
    )
    right_static_tf_node = Node(
        package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',     # x, y, z translation
                '0', '0', '0',     # roll, pitch, yaw in radians
                'world',          # parent frame
                'right_link_0'         # child frame
            ],
            output='screen'
    )
    
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager"
            ]
    )
    
    controller_arg=DeclareLaunchArgument("controller",default_value="joint_trajectory_controller",
                    description="Desired default controller. One of specified in lbr_controller.yaml")
    
    lef_arm_controller=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "controller_manager"
            ]
    )
    right_arm_controller=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "controller_manager"
            ]
    )
    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action= ros2_control_node,
            on_start=[
                joint_state_broadcaster_node,
                right_arm_controller,
                lef_arm_controller
            ])
    )
    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_moveit_config_iiwa"), "config", "moveit.rviz"
    ) 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
        ],
    )
    
    return LaunchDescription(
        [
            move_group_node,
            left_static_tf_node,
            right_static_tf_node,
            ros2_control_node,
            controller_arg,
            controller_event_handler,
            rviz_node
        ])