#ifndef JAMAL_INTERFACE_H
#define JAMAL_INTERFACE_H

#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Eigen>
#include <mblink/mblink.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>

struct JamalMotorData {
  double pos_, vel_, tau_;                 // Feedback from hardware
  double posDes_, velDes_, kp_, kd_, ff_;  // Commanded values
};

class JamalInterface : public HardwareInterface {
 public:
  JamalInterface() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const ros::Time& time, const ros::Duration& period);
  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, const ros::Time& time,
                    const ros::Duration& period);

  std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};
  std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};
  std::vector<double> kt_vec_ = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

 private:
  ros::Publisher command_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber imu_sub_;

  JamalMotorData jointData_[12]{};
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::JointState joint_state_msg_;
};

#endif  // JAMAL_INTERFACE_H
