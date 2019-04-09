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
// Updates by Tim Lee and Adithya Murali // // TBD: refactor repo

#include <stdexcept>
#include <string>

#include "dynamixel_workbench_controllers/position_control.h"

PositionControl::PositionControl()
    : node_handle_(""),
      dxl_cnt_arm_(6),
      dxl_cnt_gripper_(1),
      dxl_cnt_camera_(2) {}

PositionControl::~PositionControl()
{
  if (use_arm_)
  {
    printf("Going to rest joint... \n");
    // Going to this joint is just to avoid collisions in some edge cases
    sensor_msgs::JointState* intermediate_joint = new sensor_msgs::JointState();
    intermediate_joint->position = {0, 0.1135, 0.9603, -0.9235, -0.0015};
    goalJointPositionCallback(sensor_msgs::JointState::ConstPtr(intermediate_joint));

    usleep(5000000);

    sensor_msgs::JointState* rest_joint = new sensor_msgs::JointState();
    rest_joint->position = {0, -0.4080, 1.5079, 0.1212, 0.00767};
    goalJointPositionCallback(sensor_msgs::JointState::ConstPtr(rest_joint));

    usleep(8000000);

    for (int index = 0; index < dxl_cnt_arm_; ++index)
    {
      dxl_wb_arm_->itemWrite(dxl_id_arm_[index], "Torque_Enable", 0);
    }

    dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Torque_Enable", 0);
  }

  if (use_camera_)
  {
    for (int index = 0; index < dxl_cnt_camera_; ++index)
    {
      bool test = dxl_wb_camera_->itemWrite(dxl_id_camera_[index], "Torque_Enable", 0);
    }
  }

  ros::shutdown();
}

bool PositionControl::initLoCoBot()
{
  // LoCoBot control modes
  use_arm_ = node_handle_.param<bool>("use_arm", true);
  use_camera_ = node_handle_.param<bool>("use_camera", true);

  // Controller parameters
  device_name_   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  dxl_baud_rate_ = node_handle_.param<int>("baud_rate", 57600);

  // uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  profile_velocity_     = node_handle_.param<int>("profile_velocity", 200);
  profile_acceleration_ = node_handle_.param<int>("profile_acceleration", 50);

  if (use_arm_)
  {
    if (!initArm())
    {
      return false;
    }

    if (!initGripper())
    {
      return false;
    }
  }

  if (use_camera_)
  {
    if (!initCamera())
    {
      return false;
    }
  }

  initPublisher();
  initSubscriber();
  initServer();

  return true;
}

bool PositionControl::initArm()
{
  // Set servo IDs for arm
  dxl_id_arm_[0] = node_handle_.param<int>("joint_1_id",   1);
  dxl_id_arm_[1] = node_handle_.param<int>("joint_2_1_id", 2);
  dxl_id_arm_[2] = node_handle_.param<int>("joint_2_2_id", 3);
  dxl_id_arm_[3] = node_handle_.param<int>("joint_3_id",   4);
  dxl_id_arm_[4] = node_handle_.param<int>("joint_4_id",   5);
  dxl_id_arm_[5] = node_handle_.param<int>("joint_5_id",   6);

  dxl_wb_arm_ = new DynamixelWorkbench;

  dxl_wb_arm_->begin(device_name_.c_str(), dxl_baud_rate_);

  for (int index = 0; index < dxl_cnt_arm_; ++index)
  {
    uint16_t get_model_number;
    if (dxl_wb_arm_->ping(dxl_id_arm_[index], &get_model_number) != true)
    {
      ROS_ERROR("Arm motor not found. Please check ID and baud rate.");
      ros::shutdown();
      return false;
    }
  }

  for (int index = 0; index < dxl_cnt_arm_; ++index)
  {
    dxl_wb_arm_->jointMode(dxl_id_arm_[index], profile_velocity_, profile_acceleration_);
  }

  dxl_wb_arm_->addSyncWrite("Goal_Position");

  // Write values
  for (int index = 0; index < dxl_cnt_arm_; ++index)
  {
    dxl_wb_arm_->itemWrite(dxl_id_arm_[index], "Position_I_Gain", 5);
    dxl_wb_arm_->itemWrite(dxl_id_arm_[index], "Position_D_Gain", 50);
    dxl_wb_arm_->itemWrite(dxl_id_arm_[index], "Drive_Mode", 0);
  }

  //
  printf("-----------------------------------------------------------------------\n");
  printf("  LoCoBot Arm           \n");
  printf("-----------------------------------------------------------------------\n");
  for (int index = 0; index < dxl_cnt_arm_; ++index)
  {
    printf("MODEL   : %s\n", dxl_wb_arm_->getModelName(dxl_id_arm_[index]));
    printf("ID      : %d\n", dxl_id_arm_[index]);
    printf("Gains P : %d   I : %d   D : %d \n", dxl_wb_arm_->itemRead(dxl_id_arm_[index], "Position_P_Gain"),
      dxl_wb_arm_->itemRead(dxl_id_arm_[index], "Position_I_Gain"),
      dxl_wb_arm_->itemRead(dxl_id_arm_[index], "Position_D_Gain"));
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");

  return true;
}

bool PositionControl::initGripper()
{
  // Set servo ID of gripper
  dxl_id_gripper_[0] = node_handle_.param<int>("gripper", 7);

  dxl_wb_gripper_ = new DynamixelWorkbench;

  dxl_wb_gripper_->begin(device_name_.c_str(), dxl_baud_rate_);

  uint16_t get_model_number;
  if (dxl_wb_gripper_->ping(dxl_id_gripper_[0], &get_model_number) != true)
  {
    ROS_ERROR("Gripper motor not found. Please check ID and baud rate.");
    ros::shutdown();
    return false;
  }

  gripper_state_ = 2;
  prev_gripper_state_ = 2;
  prev_gripper_load_ = 2;

  // Write values
  dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Position_I_Gain", 5);
  dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Position_D_Gain", 50);
  dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Drive_Mode", 0);

  printf("-----------------------------------------------------------------------\n");
  printf("  LoCoBot Gripper           \n");
  printf("-----------------------------------------------------------------------\n");

  printf("MODEL   : %s\n", dxl_wb_gripper_->getModelName(dxl_id_gripper_[0]));
  printf("ID      : %d\n", dxl_id_gripper_[0]);
  printf("Gains P : %d   I : %d   D : %d \n", dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Position_P_Gain"),
    dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Position_I_Gain"),
    dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Position_D_Gain"));
  printf("-----------------------------------------------------------------------\n");

  return true;
}

bool PositionControl::initCamera()
{
  dxl_id_camera_[0] = node_handle_.param<int>("pan",  8);
  dxl_id_camera_[1] = node_handle_.param<int>("tilt", 9);

  dxl_wb_camera_ = new DynamixelWorkbench;

  dxl_wb_camera_->begin(device_name_.c_str(), dxl_baud_rate_);

  for (int index = 0; index < dxl_cnt_camera_; ++index)
  {
    uint16_t get_model_number;
    if (dxl_wb_camera_->ping(dxl_id_camera_[index], &get_model_number) != true)
    {
      ROS_ERROR("Camera motor not found. Please check ID and baud rate.");
      ros::shutdown();
      return false;
    }
  }

  for (int index = 0; index < dxl_cnt_camera_; ++index)
  {
    dxl_wb_camera_->jointMode(dxl_id_camera_[index], profile_velocity_, profile_acceleration_);
  }

  dxl_wb_camera_->addSyncWrite("Goal_Position");

  // Write values
  for (int index = 0; index < dxl_cnt_camera_; ++index)
  {
    dxl_wb_camera_->itemWrite(dxl_id_camera_[index], "Position_I_Gain", 5);
    dxl_wb_camera_->itemWrite(dxl_id_camera_[index], "Position_D_Gain", 50);
    dxl_wb_camera_->itemWrite(dxl_id_camera_[index], "Drive_Mode", 0);
  }

  printf("-----------------------------------------------------------------------\n");
  printf("  LoCoBot Pan-Tilt Camera           \n");
  printf("-----------------------------------------------------------------------\n");
  for (int index = 0; index < dxl_cnt_camera_; ++index)
  {
    printf("MODEL   : %s\n", dxl_wb_camera_->getModelName(dxl_id_camera_[index]));
    printf("ID      : %d\n", dxl_id_camera_[index]);
    printf("Gains P : %d   I : %d   D : %d \n", dxl_wb_camera_->itemRead(dxl_id_camera_[index], "Position_P_Gain"),
      dxl_wb_camera_->itemRead(dxl_id_camera_[index], "Position_I_Gain"),
      dxl_wb_camera_->itemRead(dxl_id_camera_[index], "Position_D_Gain"));
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");

  return true;
}

void PositionControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);

  if (use_arm_)
  {
    // status_pub_ = node_handle_.advertise<std_msgs::Bool>("arm/status", 10);
  }
  if (use_camera_)
  {
    pan_state_pub_ = node_handle_.advertise<std_msgs::Float64>("pan/state", 10);
    tilt_state_pub_ = node_handle_.advertise<std_msgs::Float64>("tilt/state", 10);
  }
}

void PositionControl::initSubscriber()
{
  if (use_arm_)
  {
    joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &PositionControl::goalJointPositionCallback, this);

    gripper_open_sub_ = node_handle_.subscribe("gripper/open", 10, &PositionControl::gripperOpenCallback, this);
    gripper_close_sub_ = node_handle_.subscribe("gripper/close", 10, &PositionControl::gripperCloseCallback, this);
  }
  if (use_camera_)
  {
    pan_command_sub_ = node_handle_.subscribe("pan/command", 10, &PositionControl::panCommandCallback, this);
    tilt_command_sub_ = node_handle_.subscribe("tilt/command", 10, &PositionControl::tiltCommandCallback, this);
  }
}

void PositionControl::initServer()
{
  // joint_command_server_ = node_handle_.advertiseService("joint_command", &PositionControl::jointCommandMsgCallback, this);
}

void PositionControl::dynamixelStatePublish()
{
  // dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  // dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  // for (int index = 0; index < dxl_cnt_; index++)
  // {
  //   dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
  //   dynamixel_state[index].id                  = dxl_id_[index];
  //   dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
  //   dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
  //   dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
  //   dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
  //   dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
  //   dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

  //   dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  // }
  // dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void PositionControl::gripperController()
{
  if (use_arm_)
  {
    // close gripper
    if (gripper_state_ == 1)
    {
      if (prev_gripper_state_ != gripper_state_)
      {
        // shifting to PWM mode
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Torque_Enable", 0);
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Operating_Mode", 16);
        usleep(10000);
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Torque_Enable", 1);
        prev_gripper_state_ = gripper_state_;
      }

      //printf("%d\n",dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Present_Load"));
      if (std::abs(dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Present_Load")) > GRIPPER_MAX_LOAD)
      {
        // sometime gripper gets stuck.. this part is to overcome that
        if (prev_gripper_load_ == 0)
        {
          dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Goal_PWM", -700);
          usleep(100000);
        }
        if (std::abs(dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Present_Load")) > GRIPPER_MAX_LOAD)
        {
          prev_gripper_load_ = 1;
          //printf("LOAD EXCEEDED \n");
        }
      }
      if ((dxl_wb_gripper_->itemRead(dxl_id_gripper_[0], "Present_Position") < GRIPPER_CLOSE_VALUE) || prev_gripper_load_ == 1)
      {
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Goal_PWM", 0);
        //printf("0 PWM \n");
      }
      else
      {
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Goal_PWM", -GRIPPER_PWM);
      }
    }
    // open gripper
    else if (gripper_state_ == 0)
    {
      if (prev_gripper_state_ != gripper_state_)
      {
        // shifting to position mode
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Torque_Enable", 0);
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Operating_Mode", 3);
        usleep(10000);
        dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Torque_Enable", 1);
        prev_gripper_state_ = gripper_state_;
      }
      dxl_wb_gripper_->goalPosition(dxl_id_gripper_[0], GRIPPER_OPEN_VALUE);
      prev_gripper_load_ = 0;
    }
  }
}

// void PositionControl::jointStatePublish()
// {
//   // This is for joint_state_publisher

//   int32_t present_position[dxl_cnt_] = {0, };

//   for (int index = 0; index < dxl_cnt_; index++)
//     present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

//   int32_t present_velocity[dxl_cnt_] = {0, };

//   for (int index = 0; index < dxl_cnt_; index++)
//     present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

//   sensor_msgs::JointState joint_state;
//   float velocity_motor = 0.0;
//   float position_motor = 0.0;

//   // Publish arm joint states
//   for (int index = 0; index < dxl_cnt_; index++)
//   {
//     std::stringstream joint_name;
//     if (index <= 1) {
//       joint_name << "joint_" << (index + 1);
//       position_motor = dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]);
//       velocity_motor = dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]);
//     } else if ((index == 2) || (index == 6)) {
//       continue;
//     } else {
//       joint_name << "joint_" << index;
//       position_motor = -dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]);
//       velocity_motor = -dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]);
//     }
//     joint_state.name.push_back(joint_name.str());
//     joint_state.position.push_back(position_motor);
//     joint_state.velocity.push_back(velocity_motor);
//   }

//   joint_state.header.stamp = ros::Time::now();
//   joint_states_pub_.publish(joint_state);
// }
float PositionControl::getCurrentMotorPosition(DynamixelWorkbench *dxl_wb, uint8_t id)
{
  return dxl_wb->convertValue2Radian(id, dxl_wb->itemRead(id, "Present_Position"));
}

float PositionControl::getCurrentMotorVelocity(DynamixelWorkbench *dxl_wb, uint8_t id)
{
  return dxl_wb->convertValue2Radian(id, dxl_wb->itemRead(id, "Present_Velocity"));
}

void PositionControl::getCurrentPV(std::vector<std::string>& jntNames,
                                   std::vector<float>& jntPos,
                                   std::vector<float>& jntVel,
                                   int index)
{
  int armMotorIDs[5] = {0, 1, 3, 4, 5};
  std::stringstream jointName;
  float motorPos;
  float motorVel;

  if (index <= 4)
  {
    jointName << "joint_" << (index + 1);
    if (use_arm_)
    {
      motorPos = getCurrentMotorPosition(dxl_wb_arm_, dxl_id_arm_[armMotorIDs[index]]);
      motorVel = getCurrentMotorVelocity(dxl_wb_arm_, dxl_id_arm_[armMotorIDs[index]]);
      if (index > 1)
      {
        motorPos = -motorPos;
        motorVel = -motorVel;
      }
    } else
    {
      motorPos = 0.0;
      motorVel = 0.0;
    }
  }
  else if (index > 4 && index <= 6)
  {
    if (index == 5)
    {
      jointName << "head_pan_joint";
    }
    else
    {
      jointName << "head_tilt_joint";
    }
    if (use_camera_)
    {
      motorPos = getCurrentMotorPosition(dxl_wb_camera_, dxl_id_camera_[index - 5]);
      motorVel = getCurrentMotorVelocity(dxl_wb_camera_, dxl_id_camera_[index - 5]);
    }
    else
    {
      motorPos = 0.0;
      motorVel = 0.0;
    }
  }
  else if (index > 6)
  {
    jointName << "joint_" << (index - 1);
    if (use_arm_)
    {
      motorPos = getCurrentMotorPosition(dxl_wb_gripper_, dxl_id_gripper_[0]);
      motorVel = getCurrentMotorVelocity(dxl_wb_gripper_, dxl_id_gripper_[0]);
      motorPos = (-motorPos - GRIPPER_OPEN_MOTOR_POS) /
        (GRIPPER_CLOSE_MOTOR_POS - GRIPPER_OPEN_MOTOR_POS) * (GRIPPER_CLOSE_MOVEIT - GRIPPER_OPEN_MOVEIT) + GRIPPER_OPEN_MOVEIT;
    }
    else
    {
      motorPos = GRIPPER_OPEN_MOVEIT;
      motorVel = 0.0;
    }
    if (index == 7)
    {
      motorPos = -motorPos;
      motorVel = -motorVel;
    }
  }
  else
  {
    throw std::invalid_argument("Received invalid index value");
  }
  
  jntNames[index] = jointName.str();
  jntPos[index] = motorPos;
  jntVel[index] = motorVel;
}

sensor_msgs::JointState PositionControl::getCurrentJointStates()
{
  int num_joints = dxl_cnt_arm_ + dxl_cnt_gripper_ + dxl_cnt_camera_;
  std::vector<std::string> joint_names(num_joints, "joint_0");
  std::vector<float> joint_positions(num_joints, 0);
  std::vector<float> joint_velocities(num_joints, 0);

  for (int i = 0; i < num_joints; ++i)
  {
    getCurrentPV(joint_names, joint_positions, joint_velocities, i);
  }
  sensor_msgs::JointState joint_states;
  for (int i = 0; i < num_joints; ++i)
  {
    joint_states.name.push_back(joint_names[i]);
    joint_states.position.push_back(joint_positions[i]);
    joint_states.velocity.push_back(joint_velocities[i]);
  }
  joint_states.header.stamp = ros::Time::now();
  
  return joint_states;
}

void PositionControl::jointStatePublish()
{
  sensor_msgs::JointState joint_state = getCurrentJointStates();
  joint_states_pub_.publish(joint_state);
}

void PositionControl::cameraStatePublish()
{
  if (use_camera_)
  {
    std_msgs::Float64 msg;

    // Publish camera pan state
    msg.data = getCurrentMotorPosition(dxl_wb_camera_, dxl_id_camera_[0]);
    pan_state_pub_.publish(msg);

    // Publish camera tilt state
    msg.data = getCurrentMotorPosition(dxl_wb_camera_, dxl_id_camera_[1]);
    tilt_state_pub_.publish(msg);
  }
}

void PositionControl::controlLoop()
{
  gripperController();
  dynamixelStatePublish();
  jointStatePublish();
  cameraStatePublish();
}

// bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
//                                               dynamixel_workbench_msgs::JointCommand::Response &res)
// {
//   int32_t goal_position = 0;
//   int32_t present_position = 0;

//   if (req.unit == "rad")
//   {
//     goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
//   }
//   else if (req.unit == "raw")
//   {
//     goal_position = req.goal_position;
//   }
//   else
//   {
//     goal_position = req.goal_position;
//   }

//   bool ret = dxl_wb_->goalPosition(req.id, goal_position);

//   res.result = ret;
// }

void PositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (use_arm_)
  {
    if (msg->position.size() != 5) {
      ROS_ERROR("Invalid joint state, execution aborted");
    } else {
      double goal_position[dxl_cnt_arm_] = {0.0, };

      for (int index = 0; index < 5; index++)
        goal_position[index] = msg->position.at(index);

      int32_t goal_dxl_position[dxl_cnt_arm_] = {0, };

      for (int index = 0; index < dxl_cnt_arm_; index++)
      {

        if (index <= 1) {
          // 1st (base) and 2nd joints
          goal_dxl_position[index] = dxl_wb_arm_->convertRadian2Value(dxl_id_arm_[index], goal_position[index]);
        } else if (index == 2) {
          // 2nd dual motor mirrors the first motor at index 1 (but flipped)
          goal_dxl_position[index] = dxl_wb_arm_->convertRadian2Value(dxl_id_arm_[index], -goal_position[1]);
        } else {
          // 3rd, 4th and 5th joint
          goal_dxl_position[index] = dxl_wb_arm_->convertRadian2Value(dxl_id_arm_[index], -goal_position[index-1]);
        }
      }

      dxl_wb_arm_->syncWrite("Goal_Position", goal_dxl_position);
    }
  }
}

void PositionControl::gripperCloseCallback(const std_msgs::Empty::ConstPtr &msg)
{
  if (use_arm_)
  {
    gripper_state_ = 1;
  }
}

void PositionControl::gripperOpenCallback(const std_msgs::Empty::ConstPtr &msg)
{
  if (use_arm_)
  {
    dxl_wb_gripper_->itemWrite(dxl_id_gripper_[0], "Goal_PWM", GRIPPER_PWM);
    gripper_state_ = 0;
  }
}

void PositionControl::panCommandCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (use_camera_)
  {
    dxl_wb_camera_->goalPosition(dxl_id_camera_[0],
      dxl_wb_camera_->convertRadian2Value(dxl_id_camera_[0], msg->data));
  }
}

void PositionControl::tiltCommandCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (use_camera_)
  {
    dxl_wb_camera_->goalPosition(dxl_id_camera_[1],
      dxl_wb_camera_->convertRadian2Value(dxl_id_camera_[1], msg->data));
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;

  if (!pos_ctrl.initLoCoBot())
  {
    return 0;
  }

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
