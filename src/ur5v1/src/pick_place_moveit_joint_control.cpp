#include <ros/ros.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm("manipulator");
    // moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    std::map< std::string, double > pick;
    pick = {{"shoulder_pan_joint", -1.5708}, 
              {"shoulder_lift_joint", -1.0471}, 
              {"elbow_joint", 1.5708},
              {"wrist_1_joint", -0.5409}, 
              {"wrist_2_joint", .5708}, 
              {"wrist_3_joint", 1.5707}};

    std::map< std::string, double > lean;
    lean = {{"shoulder_pan_joint", -1.5708}, 
              {"shoulder_lift_joint", -2.0943}, 
              {"elbow_joint", 2.0943},
              {"wrist_1_joint", -0.1963}, 
              {"wrist_2_joint", 1.5708}, 
              {"wrist_3_joint", 1.5708}};

    std::map< std::string, double > turn;
    turn = {{"shoulder_pan_joint", 1.5708}, 
              {"shoulder_lift_joint", -2.0943}, 
              {"elbow_joint", 2.0943},
              {"wrist_1_joint", -0.1963}, 
              {"wrist_2_joint", 1.5708}, 
              {"wrist_3_joint", 1.5708}};

    std::map< std::string, double > place;
    place = {{"shoulder_pan_joint", 1.5708}, 
              {"shoulder_lift_joint", -1.0471}, 
              {"elbow_joint", 1.5709},
              {"wrist_1_joint", -0.5237}, 
              {"wrist_2_joint", 1.5708}, 
              {"wrist_3_joint", 1.5708}};

    ROS_INFO_NAMED("ENPM662", "Pick and Place Started");
    move_group_interface_arm.setJointValueTarget(pick);
    ROS_INFO_NAMED("ENPM662", "Pick : Set joint values completed.");
    ROS_INFO_NAMED("ENPM662", "Pick : Planning Started");
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ENPM662", "Pick : Planning Completed");
    ROS_INFO_NAMED("ENPM662", "Pick : Motion Started.");
    move_group_interface_arm.move();
    ROS_INFO_NAMED("ENPM662", "Pick : Motion Completed");

    move_group_interface_arm.setJointValueTarget(lean);
    ROS_INFO_NAMED("ENPM662", "Retract : Set joint values completed.");
    ROS_INFO_NAMED("ENPM662", "Retract : Planning Started");
     success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ENPM662", "Retract : Planning Completed");
    ROS_INFO_NAMED("ENPM662", "Retract : Motion Started.");
    move_group_interface_arm.move();
    ROS_INFO_NAMED("ENPM662", "Retract : Motion Completed");


    move_group_interface_arm.setJointValueTarget(turn);
    ROS_INFO_NAMED("ENPM662", "Turn : Set joint values completed.");
    ROS_INFO_NAMED("ENPM662", "Turn : Planning Started");
     success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ENPM662", "Turn : Planning Completed");
    ROS_INFO_NAMED("ENPM662", "Turn : Motion Started.");
    move_group_interface_arm.move();
    ROS_INFO_NAMED("ENPM662", "Turn : Motion Completed");

    move_group_interface_arm.setJointValueTarget(place);
    ROS_INFO_NAMED("ENPM662", "Place : Set joint values completed.");
    ROS_INFO_NAMED("ENPM662", "Place : Planning Started");
     success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ENPM662", "Place : Planning Completed");
    ROS_INFO_NAMED("ENPM662", "Place : Motion Started.");
    move_group_interface_arm.move();
    ROS_INFO_NAMED("ENPM662", "Place : Motion Completed");

    ROS_INFO_NAMED("ENPM662", "Pick and Place Completed");
    
    
    //1. Move to pick location:
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    // bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group_interface_arm.move();

    // //2. Move to lean location:
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("up"));
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group_interface_arm.move();

    // //3. Move to turn location:
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("turn"));
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group_interface_arm.move();

    // //4. Move to place location:
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("place"));
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group_interface_arm.move();
    
  ros::waitForShutdown();
  return 0;
}