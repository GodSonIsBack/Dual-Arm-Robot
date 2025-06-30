#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("move_program");
    auto arm_type=node->declare_parameter("arm_type","left");
    node->declare_parameter("obj_en",false);
    std::string arm= node->get_parameter("arm_type").as_string()+ "_arm";

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});
    auto MoMoveGroupInterface = moveit::planning_interface::MoveGroupInterface(
      node, arm);
    auto logger = node->get_logger();

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        MoMoveGroupInterface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Create closures for visualization
    auto const draw_title = [&moveit_visual_tools,arm](auto text) {
        auto const text_pose = [arm] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; 
            (arm=="left_arm")?msg.translation().y() = -1.5 : msg.translation().y()=1.5; // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);};
    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);};
    auto const draw_trajectory_tool_path =[&moveit_visual_tools,jmg = MoMoveGroupInterface.getRobotModel()->getJointModelGroup(arm)](auto const trajectory) 
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

    //setting target pose
    auto target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.y = 0.8;
    msg.orientation.w = 0.6;
    msg.position.x = 0.1;
    msg.position.y = 0.4;
    msg.position.z = 0.4;
    return msg;
    }();
    if(arm=="left_arm") target_pose.position.y-=1.5;
    else if (arm=="right_arm") target_pose.position.y+=1.5;
    else {RCLCPP_ERROR(logger,"NIGGA SET A CORRECT ARM!!!");rclcpp::shutdown();return 0;}

    MoMoveGroupInterface.setPoseTarget(target_pose);

    
    //Create collision object for the robot to avoid
    if((node->get_parameter("obj_en").as_bool()))
    {auto const collision_object = [frame_id =MoMoveGroupInterface.getPlanningFrame(),arm] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;
        
        //size of box
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.5;
        
        //pose 
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0; 
        box_pose.position.x = 0.2;
        box_pose.position.y = 0.2;
        (arm=="left_arm") ? box_pose.position.y-=1.5 : box_pose.position.y+=1.5;
        box_pose.position.z = 0.25;
        
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }();
    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}


    //PLANNING
    MoMoveGroupInterface.setNumPlanningAttempts(100);
    MoMoveGroupInterface.setMaxVelocityScalingFactor(0.2);
    MoMoveGroupInterface.setMaxAccelerationScalingFactor(0.3);

    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();

    auto const [success, plan]= [&MoMoveGroupInterface]()
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto const ok=static_cast<bool>(MoMoveGroupInterface.plan(my_plan));
        return std::make_pair(ok,my_plan); 
    }();


    if(success)
    {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        MoMoveGroupInterface.execute(plan);
    }else
    {
        draw_title("FAILED PLANNING");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    rclcpp::shutdown();
    spinner.join();
}
