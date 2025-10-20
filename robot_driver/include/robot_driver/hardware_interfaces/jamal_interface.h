
// #ifndef JAMAL_INTERFACE_H
// #define JAMAL_INTERFACE_H

// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/JointState.h>
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

// #include <eigen3/Eigen/Eigen>
// #include <quad_msgs/LegCommandArray.h>
// #include <robot_driver/hardware_interfaces/hardware_interface.h>

// #include <string>
// #include <vector>

// struct JamalMotorData {
//   double pos_, vel_, tau_;                 // feedback (optional)
//   double posDes_, velDes_, kp_, kd_, ff_;  // commands
// };

// class JamalInterface : public HardwareInterface {
//  public:
//   JamalInterface();

//   // NOTE: Base class in your tree doesn't declare these as virtual,
//   // so do NOT use 'override' here.
//   void init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
//   void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
//   void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

//   // These DO override virtuals in the base class
//   bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
//             const ros::Time& time, const ros::Duration& period) override;

//   bool recv(sensor_msgs::JointState& joint_state_msg,
//             sensor_msgs::Imu& imu_msg,
//             const ros::Time& time, const ros::Duration& period) override;

//  private:
//   // ROS I/O
//   ros::Publisher  command_pub_;
//   ros::Subscriber joint_state_sub_;
//   ros::Subscriber imu_sub_;

//   // Cached messages
//   sensor_msgs::Imu        imu_msg_;
//   sensor_msgs::JointState joint_state_msg_;

//   // Config
//   std::vector<std::string> joint_names_ = {"8","0","1","9","2","3","10","4","5","11","6","7"}; // replace with real names via params later
//   std::vector<double>      kt_vec_      = std::vector<double>(12, 1.0);

//   // Command accumulation
//   JamalMotorData jointData_[12]{};

//   // Keep trajectory time strictly increasing across cycles
//   ros::Duration traj_time_from_start_{0.0};

//   // Topics (param-overridable in init)
//   std::string imu_topic_    = "/robot_1/trunk_imu";
//   std::string joints_topic_ = "/robot_1/joint_states";
//   std::string traj_topic_   = "/joint_trajectory_controller/command"; // common default
// };

// #endif  // JAMAL_INTERFACE_H
