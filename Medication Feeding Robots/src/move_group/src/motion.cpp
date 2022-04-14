#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/Int32.h"
#include <math.h> 


// global variables
int x, y = 0;
double forward = 0;
double spoon_dist = 0.1339;  // in meters (without scooping part)
int x_second, y_second = 0;
int x_px_diff = 0;
bool no_face_detected = false;
bool end_process = false;

// callback functions
void distX(const std_msgs::Int32::ConstPtr& listened_x)  //const std_msgs::String::ConstPtr& msg
{
    if (x == 0) {
	no_face_detected = false;
        x = listened_x->data;  // initial x
    }
    else if (x != 0  && x != -10000) {
	no_face_detected = false;
	x_second = listened_x->data;  // second point of x (the pixels it went over to the other side)
	x_px_diff = x_second - x;  // the total number of pixels moved by 0.05m
    }

    // if a face is not detected
    if (listened_x->data == -10000) {
	no_face_detected = true;
	//ROS_INFO("I heard x: [%i]", x);
    }
    //ROS_INFO("I heard x: [%i]", x);
}

void distY(const std_msgs::Int32::ConstPtr& listened_y)
{
    if (y == 0) {
	no_face_detected = false;
        y = listened_y->data;
    }
    else if (y != 0  && y != -10000) {
	no_face_detected = false;
	y_second = listened_y->data;  // second point of y (the pixels it went over to below face)
    }

    // if a face is not detected
    if (listened_y->data == -10000) {
	no_face_detected = true;
	//ROS_INFO("I heard y: [%i]", y);
    }
    //ROS_INFO("I heard y: [%i]", y);
}

void distForward(const std_msgs::Int32::ConstPtr& listened_forward)
{
    forward = listened_forward->data;
    //ROS_INFO("I heard forward: [%f]", forward);
}


// main function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // listens to x, y distances needed to move
  ros::Subscriber sub_x = n.subscribe("chatter", 1, distX);
  ros::Subscriber sub_y = n.subscribe("chatter2", 1, distY);
  ros::Subscriber sub_forward = n.subscribe("chatter3", 1, distForward);
  
  ros::Duration(0.5).sleep(); // wait for 0.5 seconds to ensure x, y get necessary values


  // -------------------- MOVE IN SIDE (RIGHT/LEFT) DIRECTION --------------------
  // NOTE:
  // +X IS FORWARD
  // +Y IS LEFT
  // +Z IS UP

  // get current pose of robotic arm
  geometry_msgs::Pose home_pose = move_group.getCurrentPose().pose; // save home configuration
  geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  //waypoints.push_back(target_pose);  // push in initial pose

  // if a face is not detected, stay in home configuration
  if (no_face_detected == true) {
    target_pose = home_pose;
    ROS_INFO("Face is not detected, ending the process... Robotic arm staying at home");
    end_process = true;
  }
  // if a face is detected
  else {
    // move in side (+-y) direction
    if (x < 0) {
      target_pose.position.y -= -0.1;
    }
    else if (x > 0) {
      target_pose.position.y -= 0.1;
    }
  }
  
  waypoints.push_back(target_pose);  // right/left

  // reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan);

  ros::Duration(1).sleep(); // wait for 1 seconds

  // end the process if no face is detected
  if (end_process) {
    ros::shutdown();
  }


  // -------------------- CALIBRATE IN SIDE (RIGHT/LEFT) DIRECTION --------------------
  // listens to face detection node again to ensure face is detected
  //sub_x = n.subscribe("chatter", 1, distX);
  //sub_y = n.subscribe("chatter2", 1, distY);

  // get current pose of robotic arm
  target_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints_2;

  // compute meters robots moved per pixel
  double m_per_px = 0.1 / (1.0 * x_px_diff); 

  // if a face is not detected, return to home configuration
  if (no_face_detected == true) {
    target_pose = home_pose;
    ROS_INFO("Face is not detected, ending the process... Robotic arm returning to home");
    end_process = true;
  }
  // if a face is detected
  else {
    // change to absolute value
    if (m_per_px < 0) {
       m_per_px = -m_per_px;
    }
    // calibrate back to center of face in side (+-y) direction
    double dist = m_per_px * (1.0 * x_second);
    target_pose.position.y -= dist;
  }

  waypoints_2.push_back(target_pose);  // right/left

  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
  moveit_msgs::RobotTrajectory trajectory_2;
  fraction = move_group.computeCartesianPath(waypoints_2, eef_step, jump_threshold, trajectory_2);
  my_plan_2.trajectory_ = trajectory_2;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints_2, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints_2.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints_2[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan_2);

  ros::Duration(1).sleep(); // wait for 1 seconds

  // end the process if no face is detected
  if (end_process) {
    ros::shutdown();
  }


  // -------------------- MOVE IN UP/DOWN DIRECTION --------------------
  // listens to face detection node again to ensure face is detected
  //sub_x = n.subscribe("chatter", 1, distX);
  //sub_y = n.subscribe("chatter2", 1, distY);

  // get current pose of robotic arm
  target_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints_3;
  
  // does not need to test if a face is detected
  // because the code could not execute this far if a face is
  // not previously detected

  // move in down (+-z) direction
  if (y < 0) {
    target_pose.position.z -= -0.06;
  }
  else if (y > 0) {
    target_pose.position.z -= 0.06;
  }

  waypoints_3.push_back(target_pose);  // up/down


  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_3;
  moveit_msgs::RobotTrajectory trajectory_3;
  fraction = move_group.computeCartesianPath(waypoints_3, eef_step, jump_threshold, trajectory_3);
  my_plan_3.trajectory_ = trajectory_3;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints_3, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints_3.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints_3[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan_3);

  ros::Duration(1).sleep(); // wait for 1 seconds

  // end the process if no face is detected
  if (end_process) {
    ros::shutdown();
  }


  // -------------------- CALIBRATE IN UP/DOWN DIRECTION --------------------
  // listens to face detection node again to ensure face is detected
  //sub_x = n.subscribe("chatter", 1, distX);
  //sub_y = n.subscribe("chatter2", 1, distY);

  // get current pose of robotic arm
  target_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints_4;

  // if a face is not detected, return to home configuration
  if (no_face_detected == true) {
    target_pose = home_pose;
    ROS_INFO("Face is not detected, ending the process... Robotic arm returning to home");
    end_process = true;
  }
  // if a face is detected
  else {
    double dist = m_per_px * (1.0 * y_second);
    target_pose.position.z -= dist;
    target_pose.position.z -= 0.015;  // account for the height between the spoon and camera
  }

  waypoints_4.push_back(target_pose);  // up/down


  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_4;
  moveit_msgs::RobotTrajectory trajectory_4;
  fraction = move_group.computeCartesianPath(waypoints_4, eef_step, jump_threshold, trajectory_4);
  my_plan_4.trajectory_ = trajectory_4;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints_4, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints_4.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints_4[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan_4);

  ros::Duration(1).sleep(); // wait for 1 seconds

  // end the process no face is detected
  if (end_process) {
    ros::shutdown();
  }

  // -------------------- MOVE IN FORWARD DIRECTION --------------------
  // listens to face detection node again to ensure face is detected
  //sub_x = n.subscribe("chatter", 1, distX);
  //sub_y = n.subscribe("chatter2", 1, distY);

  // get current pose of robotic arm
  target_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints_5;
  //waypoints_3.push_back(target_pose);  // push in initial pose

  // move in forward (+x) direction
  //target_pose.position.x += 0.15;  // move forward 15cm to feed medication
  //ROS_INFO("I heard original forward distance (m) is: [%f]", forward/1000);  //"forward" is in mm
  //ROS_INFO("I heard adjusted forward distance (m) is: [%f]", forward/1000 - spoon_dist);  //"forward" is in mm
  if (forward == 0) {
    target_pose.position.x += 0.15;  // move a fixed distance of 15cm forward to feed medication
    ROS_INFO("Oh no, I heard forward distance is: [%.2f]", 0.15);
  }
  else {
    //ROS_INFO("I heard forward distance is: [%.2f%%]", forward);
    target_pose.position.x += forward/1000 - spoon_dist;
    ROS_INFO("I heard forward distance is: [%.2f]", forward/1000 - spoon_dist);
  }
  waypoints_5.push_back(target_pose);


  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_5;
  moveit_msgs::RobotTrajectory trajectory_5;
  fraction = move_group.computeCartesianPath(waypoints_5, eef_step, jump_threshold, trajectory_5);

  //ROS_INFO("I heard fraction is: [%.2f%%]", fraction * 100.0);
  //ROS_INFO(fraction);

  // ensure niryo can reach the forward distance, if not, move a fixed 15cm
  /*if (fraction != 1)
  {
    waypoints_5.pop_back();  // remove the original target pose
    // get current pose of robotic arm
    target_pose = move_group.getCurrentPose().pose;
    target_pose.position.x += 0.15;
    waypoints_5.push_back(target_pose); 
    fraction = move_group.computeCartesianPath(waypoints_5, eef_step, jump_threshold, trajectory_5);
    //ROS_INFO("I heard fraction is now: [%.2f%%]", fraction * 100.0);
  }*/

  my_plan_5.trajectory_ = trajectory_5;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints_5, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints_5.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints_5[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan_5);

  ros::Duration(5).sleep(); // wait for 5 seconds


  // -------------------- MOVE BACK TO HOME CONFIGURATION --------------------
  std::vector<geometry_msgs::Pose> waypoints_6;

  // push in home configuration pose
  waypoints_6.push_back(home_pose);


  // specify 0.01 as the max step in Cartesian translation
  // jump threshold as 0.0
  // WARNING - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_6;
  moveit_msgs::RobotTrajectory trajectory_6;
  fraction = move_group.computeCartesianPath(waypoints_6, eef_step, jump_threshold, trajectory_6);
  my_plan_6.trajectory_ = trajectory_6;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints_6, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints_6.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints_6[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  // execute motion in Gazebo
  move_group.execute(my_plan_6);

  ros::Duration(1).sleep(); // wait for 1 seconds

  // finish process
  ros::shutdown();
  return 0;
}
