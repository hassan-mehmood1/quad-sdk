#include "robot_driver/hardware_interfaces/hardware_interface.h"

// Define constructor (even if empty)
HardwareInterface::HardwareInterface() = default;

// Define virtual destructor (important!)
// HardwareInterface::~HardwareInterface() = default;

// Provide empty default implementations for virtual functions
bool HardwareInterface::send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                             const ros::Time& time,
                             const ros::Duration& period) {
  return true;
}

bool HardwareInterface::recv(sensor_msgs::JointState& joint_state_msg,
                             sensor_msgs::Imu& imu_msg,
                             const ros::Time& time,
                             const ros::Duration& period) {
  return true;
}

void HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Optional base initialization
}
