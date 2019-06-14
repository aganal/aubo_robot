/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 *
 *  Author: zhaoyu
 *  email : zhaoyu@aubo-robotics.cn
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <industrial_msgs/RobotStatus.h>
#include <aubo_msgs/JointMsg.h>
#include <aubo_msgs/Vector3.h>
#include <aubo_msgs/WayPoint.h>
#include "std_msgs/String.h"

void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  ROS_INFO("protective stop flag: %d", msg->in_error.val);
}

void jointMsgCB(const aubo_msgs::JointMsg::ConstPtr &msg)
{
  ROS_INFO("actual_current: %d", msg->actual_current[1]);
}

void wayPointCB(const aubo_msgs::WayPoint::ConstPtr &msg)
{
  ROS_INFO("actual_rotation.x: %f", msg->actual_rotation.x);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveGroupInterface_To_Kinetic");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh;
  
  ros::Publisher robot_control_pub_ = nh.advertise<std_msgs::String>("robot_control" ,100);
  // poweOn
  // sleep(1);
  // std_msgs::String cmd;
  // cmd.data = "powerOn";
  // robot_control_pub_.publish(cmd);

  ros::Subscriber sub_robot_status_ = nh.subscribe("robot_status", 1, robotStatusCB);
  ros::Subscriber sub_joint_msg_ = nh.subscribe("/aubo_driver/joint_msgs", 1, jointMsgCB);
  ros::Subscriber sub_wayPoint_ = nh.subscribe("/aubo_driver/waypoint", 1, wayPointCB);

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator";

  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //joint Position
  std::vector<double> home_position;
  home_position.push_back(-0.652182);
  home_position.push_back(-1.518637);
  home_position.push_back(-0.465616);
  home_position.push_back(-0.718959);
  home_position.push_back(-1.553274);
  home_position.push_back(-0.096143); 
  move_group.setJointValueTarget(home_position);
  move_group.move();

  sleep(5);

  // target pose
  tf::Quaternion q;
  q.setRPY(-0.201168, -0.002089, -2.320873);
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.409621;
  target_pose1.position.y = -0.464676;
  target_pose1.position.z = 0.079343;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Perform planning actions
  move_group.execute(my_plan);

  sleep(5);

  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();

  sleep(5);

  //waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.z -= 0.2;
  waypoints.push_back(target_pose2);  // down

  target_pose2.position.y -= 0.15;
  waypoints.push_back(target_pose2);  // right

  target_pose2.position.z += 0.2;
  target_pose2.position.y += 0.2;
  target_pose2.position.x -= 0.2;
  waypoints.push_back(target_pose2);  // up and left

  // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
  move_group.setMaxVelocityScalingFactor(0.5);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;           //(The jump threshold is set to 0.0)
  const double eef_step = 0.01;                //(interpolation step)

  // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Move to the home point position
  my_plan.trajectory_= trajectory;
  move_group.execute(my_plan);

  sleep(1);

  // cmd.data = "powerOff";
  // robot_control_pub_.publish(cmd);

  spinner.stop();

  ros::spin();
  return 0;
}
