#include "robot_driver/hardware_interfaces/jamal_interface.h"
#include "legged_unitree_hw/UnitreeHW.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

bool JamalInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

  // Subscribe to IMU data
  imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("/imu/data_raw", 10, &JamalInterface::imuCallback, this);
  joint_state_sub_ = root_nh.subscribe<sensor_msgs::JointState>("/motor_states", 10, &JamalInterface::jointStateCallback, this);
  command_pub_ = root_nh.advertise<trajectory_msgs::JointTrajectory>("/joint_controller/command", 10);

  return true;
}

void JamalInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  imu_msg_ = *msg;

}

void JamalInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Resize once to 12
  joint_state_msg_.name.resize(12);
  joint_state_msg_.position.resize(12);
  joint_state_msg_.velocity.resize(12);
  joint_state_msg_.effort.resize(12);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& name = msg->name[i];

    for (int j = 0; j < 12; ++j) {
      if (joint_names_[j] == name) {
        joint_state_msg_.name[j] = name;
        joint_state_msg_.position[j] = msg->position[i];
        joint_state_msg_.velocity[j] = msg->velocity[i];
        joint_state_msg_.effort[j] = kt_vec_[j] * msg->effort[i];
      }
    }
  }
}


bool JamalInterface::recv(sensor_msgs::JointState& joint_state_msg,
                          sensor_msgs::Imu& imu_msg,
                          const ros::Time& time,
                          const ros::Duration& /*period*/) {
  joint_state_msg = joint_state_msg_;
  imu_msg = imu_msg_;

  std::ostringstream oss;
  oss << "Received Joint States (from /motor_states):\n";
  for (int i = 0; i < 12; ++i) {
    oss << "Joint " << i << " (" << joint_state_msg.name[i] << "): "
        << "pos = " << joint_state_msg.position[i] << ", "
        << "vel = " << joint_state_msg.velocity[i] << ", "
        << "tau = " << joint_state_msg.effort[i] << "\n";
  }
  ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
  return true;
}


bool JamalInterface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg, 
  const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();

  // One trajectory point containing all desired values
  trajectory_msgs::JointTrajectoryPoint point;

  traj_msg.joint_names = joint_names_;
  int leg_command_heartbeat = 1;

  int a = 0; /// very important
  for (int i = 0; i < 4; ++i) {  // For each leg
  // std::cout << "leg = " << i << std::endl;
  quad_msgs::LegCommand leg_command =
      last_leg_command_array_msg.leg_commands.at(i);

  for (int j = 0; j < 3; ++j) {  // For each joint
    // std::cout << "joint = " << j << std::endl;
    jointData_[a].posDes_ = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
    jointData_[a].velDes_ = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
    jointData_[a].ff_     = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
    a = a+1;
  }
}
  point.positions.clear();
  point.velocities.clear();
  point.effort.clear();
  // Positions 
  for (int i = 0; i < 12; ++i)
    point.positions.push_back(jointData_[i].posDes_);

  // Velocities
  for (int i = 0; i < 12; ++i)
    point.velocities.push_back(jointData_[i].velDes_);

  // Efforts (using ff_ for feedforward torques/forces)
  for (int i = 0; i < 12; ++i)
    point.effort.push_back(jointData_[i].ff_);

  // Set time_from_start (required field)
  point.time_from_start = ros::Duration(0.01); // e.g., 10ms

  // Add the point to trajectory
  traj_msg.points.push_back(point);

  // Publish
  command_pub_.publish(traj_msg);


  ROS_INFO_STREAM_THROTTLE(0.5,
    "Publishing Joint Commands (posDes, velDes, ff):\n" <<
    [&]() {
    std::ostringstream oss;
    for (int i = 0; i < 12; ++i) {
      oss << "Joint " << i << ": ["
          << jointData_[i].posDes_ << ", "
          << jointData_[i].velDes_ << ", "
          << jointData_[i].ff_ << "]\n";
    }
    return oss.str();
    }()
    );
    return true;
}
