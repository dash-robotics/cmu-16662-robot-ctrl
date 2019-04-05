/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <ros/ros.h>

#include "message_header.h"

#include <sensor_msgs/JointState.h>
// #include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

// Probably should make these constexpr
#define GRIPPER_OPEN_MOTOR_POS -1.147412
#define GRIPPER_CLOSE_MOTOR_POS 0.84368409
#define GRIPPER_OPEN_MOVEIT -0.025
#define GRIPPER_CLOSE_MOVEIT -0.002
#define GRIPPER_OPEN_VALUE 2800
#define GRIPPER_CLOSE_VALUE 1800
#define GRIPPER_MAX_LOAD 500
#define GRIPPER_PWM 500

class PositionControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publishers
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;
  // ros::Publisher status_pub_;
  ros::Publisher pan_state_pub_;
  ros::Publisher tilt_state_pub_;

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;
  ros::Subscriber gripper_open_sub_;
  ros::Subscriber gripper_close_sub_;
  ros::Subscriber pan_command_sub_;
  ros::Subscriber tilt_command_sub_;

  // ROS Service Server
  ros::ServiceServer joint_command_server_;

  // ROS Service Client

  // Dynamixel Workbench (old)
  // DynamixelWorkbench *dxl_wb_;
  // uint8_t dxl_id_[16];
  // uint8_t dxl_cnt_;

  // Dynamixel Workbench for arm
  DynamixelWorkbench *dxl_wb_arm_;
  uint8_t dxl_id_arm_[6];
  uint8_t dxl_cnt_arm_;

  // Dynamixel Workbench for gripper
  DynamixelWorkbench *dxl_wb_gripper_;
  uint8_t dxl_id_gripper_[1];
  uint8_t dxl_cnt_gripper_;
  uint8_t gripper_state_;
  uint8_t prev_gripper_state_;
  uint8_t prev_gripper_load_;

  // Dynamixel Workbench for camera
  DynamixelWorkbench *dxl_wb_camera_;
  uint8_t dxl_id_camera_[2];
  uint8_t dxl_cnt_camera_;

  // Platforn configuration
  bool use_arm_;
  bool use_camera_;

  // Controller parameters
  std::string device_name_;
  uint32_t dxl_baud_rate_;
  uint32_t profile_velocity_;
  uint32_t profile_acceleration_;

 public:
  PositionControl();
  ~PositionControl();
  void controlLoop(void);
  bool initLoCoBot();

 private:
  bool initArm();
  bool initGripper();
  bool initCamera();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();
  void jointStatePublish();
  sensor_msgs::JointState getCurrentJointStates();
  float getCurrentMotorPosition(DynamixelWorkbench *dxl_wb, uint8_t id);
  float getCurrentMotorVelocity(DynamixelWorkbench *dxl_wb, uint8_t id);
  void getCurrentPV(std::vector<std::string>& jntNames,
    std::vector<float>& jntPos, std::vector<float>& jntVel, int index);
  void cameraStatePublish();
  void gripperController();

  void initServer();
  // bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               // dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void gripperCloseCallback(const std_msgs::Empty::ConstPtr &msg);
  void gripperOpenCallback(const std_msgs::Empty::ConstPtr &msg);
  void panCommandCallback(const std_msgs::Float64::ConstPtr &msg);
  void tiltCommandCallback(const std_msgs::Float64::ConstPtr &msg);
};

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
