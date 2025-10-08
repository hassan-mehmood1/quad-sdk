#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <eigen_conversions/eigen_msg.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>
#define MATH_PI 3.141592

//! Implements an abstract class for robot hardware interfaces.
/*!
   HardwareInterface provides an abstract robot hardware interface class. The
   virtual functions declared here must be implemented by the derived class.
*/
class HardwareInterface {
 public:
  /**
   * @brief Constructor for HardwareInterface
   * @return Constructed object of type HardwareInterface
   */
  HardwareInterface();
  virtual ~HardwareInterface() = default; 

  /**
   * @brief Load the hardware interface
   * @param[in] argc Argument count
   * @param[in] argv Argument vector
   */
//   virtual void loadInterface(int argc, char** argv) = 0;

  /**
   * @brief Unload the hardware interface
   */
//   virtual void unloadInterface() = 0;
//   bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
//   virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) = 0;
  void init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  /**
   * @brief Send commands to the robot
   * @param[in] leg_command_array_msg Message containing leg commands
   * @param[in] user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
//   virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
//                     const Eigen::VectorXd& user_tx_data) = 0;
  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const ros::Time& time, const ros::Duration& period);

  /**
   * @brief Recieve data from the robot
   * @param[out] joint_state_msg Message containing joint state information
   * @param[out] imu_msg Message containing imu information
   * @param[out] user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
//   virtual bool recv(sensor_msgs::JointState& joint_state_msg,
//                     sensor_msgs::Imu& imu_msg,
//                     Eigen::VectorXd& user_rx_data) = 0;
  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, const ros::Time& time,
                    const ros::Duration& period);
//  private:
//   ros::Publisher command_pub_;
//   ros::Subscriber joint_state_sub_;
//   ros::Subscriber imu_sub_;
//   sensor_msgs::Imu imu_msg_;
//   sensor_msgs::JointState joint_state_msg_;

 protected:
};

#endif  // HARDWARE_INTERFACE_H
