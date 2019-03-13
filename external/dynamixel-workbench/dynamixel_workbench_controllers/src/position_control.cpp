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

#include <stdexcept>
#include <string>

#include "dynamixel_workbench_controllers/position_control.h"

PositionControl::PositionControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Position");

  initPublisher();
  initSubscriber();
  initServer();
}

PositionControl::~PositionControl()
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

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void PositionControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench controller; position control example             \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  // Setting some placeholder I-D gains now, there are default P gains
  for (int index = 0; index < dxl_cnt_; index++)
  {
    bool write_pos_i_gain = dxl_wb_->itemWrite(dxl_id_[index], "Position_I_Gain", 5);
    bool write_pos_d_gain = dxl_wb_->itemWrite(dxl_id_[index], "Position_D_Gain", 50);
    bool write_drive_mode = dxl_wb_->itemWrite(dxl_id_[index], "Drive_Mode", 0);

    // Assert the values have actually been written
    if (!write_pos_i_gain)
    {
      throw std::runtime_error(std::string("Could not write register \"Position_I_Gain\" for servo " +
                                           std::to_string(index)).c_str());
    }

    if (!write_pos_d_gain)
    {
      throw std::runtime_error(std::string("Could not write register \"Position_D_Gain\" for servo " +
                                           std::to_string(index)).c_str());
    }

    if (!write_drive_mode)
    {
      throw std::runtime_error(std::string("Could not write register \"Drive_Mode\" for servo " +
                                           std::to_string(index)).c_str());
    }
  }


  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("Gains P : %d   I : %d   D : %d \n", dxl_wb_->itemRead(dxl_id_[index], "Position_P_Gain"),
      dxl_wb_->itemRead(dxl_id_[index], "Position_I_Gain"),
      dxl_wb_->itemRead(dxl_id_[index], "Position_D_Gain"));
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void PositionControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
}

void PositionControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &PositionControl::goalJointPositionCallback, this);
}

void PositionControl::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("joint_command", &PositionControl::jointCommandMsgCallback, this);
}

void PositionControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void PositionControl::jointStatePublish()
{
  // This is for joint_state_publisher

  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

  sensor_msgs::JointState joint_state;
  float velocity_motor = 0.0;
  float position_motor = 0.0;

  // Publish arm joint states
  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream joint_name;
    if (index <= 1) {
      joint_name << "joint_" << (index + 1);
      position_motor = dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]);
      velocity_motor = dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]);
    } else if ((index == 2) || (index == 6)) {
      continue;
    } else {
      joint_name << "joint_" << index;
      position_motor = -dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]);
      velocity_motor = -dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]);
    }
    joint_state.name.push_back(joint_name.str());
    joint_state.position.push_back(position_motor);
    joint_state.velocity.push_back(velocity_motor);
  }

  joint_state.header.stamp = ros::Time::now();
  joint_states_pub_.publish(joint_state);
}

void PositionControl::controlLoop()
{
  dynamixelStatePublish();
  jointStatePublish();
}

bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;

  if (req.unit == "rad")
  {
    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
  }
  else if (req.unit == "raw")
  {
    goal_position = req.goal_position;
  }
  else
  {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

  res.result = ret;
}

void PositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->position.size() != 5) {
    ROS_ERROR("Invalid joint state, execution aborted");
  } else {
    double goal_position[dxl_cnt_] = {0.0, };

    for (int index = 0; index < 5; index++)
      goal_position[index] = msg->position.at(index);

    int32_t goal_dxl_position[dxl_cnt_] = {0, };

    for (int index = 0; index < dxl_cnt_; index++)
    {

      if (index <= 1) {
        // 1st (base) and 2nd joints
        goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
      } else if (index == 2) {
        // 2nd dual motor mirrors the first motor at index 1 (but flipped)
        goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], -goal_position[1]);
      } else {
        // 3rd, 4th and 5th joint
        goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], -goal_position[index-1]);
      }
    }

    dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
