import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    
    arm_type_arg=DeclareLaunchArgument("arm_type",default_value="left",
                description="Choosing which arm to control")
    obj_en_arg=DeclareLaunchArgument("obj_en",default_value="false",
                description="whether to enable collision object in planning scene")
    
    
    dual_arm_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("dual_arm_moveit_config"),
                         "launch","demo.launch.py")),
    )
    
    # move_program_node=Node(
    #     package="move_program",
    #     executable="move_program",
    #     parameters=[{
    #         "arm_type":LaunchConfiguration("arm_type"),
    #         "obj_en":LaunchConfiguration("obj_en")
    #         }]
    # )
    move_program_cmd = (
        'ros2 run move_program move_program ',
        '--ros-args --param arm_type:=', LaunchConfiguration("arm_type"),
        ' --param obj_en:=',LaunchConfiguration("obj_en")
    )
    move_program_node = ExecuteProcess(
        cmd=[
            "terminator", "-x", "bash", "-c", move_program_cmd
        ],
        shell=False
    )
    
    return LaunchDescription(
        [
            arm_type_arg,
            obj_en_arg,
            dual_arm_launch,
            move_program_node
        ]
        )