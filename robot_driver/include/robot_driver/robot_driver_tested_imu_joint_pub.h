// #ifndef ROBOT_DRIVER_H
// #define ROBOT_DRIVER_H

// #include <ros/ros.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/UInt8.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <quad_msgs/GRFArray.h>
// #include <quad_msgs/LegCommandArray.h>
// #include <quad_msgs/RobotPlan.h>
// #include <quad_msgs/RobotState.h>

// #include <eigen3/Eigen/Eigen>
// #include <eigen_conversions/eigen_msg.h>

// #include <memory>
// #include <string>
// #include <vector>
// #include <limits>

// #include <quad_utils/ros_utils.h>
// #include <quad_utils/math_utils.h>

// #include "robot_driver/controllers/grf_pid_controller.h"
// #include "robot_driver/controllers/inverse_dynamics_controller.h"
// #include "robot_driver/controllers/joint_controller.h"
// #include "robot_driver/estimators/comp_filter_estimator.h"
// #include "robot_driver/estimators/ekf_estimator.h"
// #include "robot_driver/estimators/state_estimator.h"
// #include "robot_driver/estimators/unitree_estimator.h"
// #include "robot_driver/hardware_interfaces/hardware_interface.h"
// #include "robot_driver/hardware_interfaces/jamal_interface.h"
// #include "robot_driver/robot_driver_utils.h"

// class RobotDriver {
//  public:
//   RobotDriver(ros::NodeHandle nh, int argc, char** argv);
//   void spin();

//  private:
//   void initLegController();
//   void initStateControlStructs();
//   void initStateEstimator();

//   void controlModeCallback(const std_msgs::UInt8::ConstPtr& msg);
//   void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg);
//   void robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg);
//   void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
//   void trajectoryStateCallback(const quad_msgs::RobotState::ConstPtr& msg);
//   void singleJointCommandCallback(const geometry_msgs::Vector3::ConstPtr& msg);
//   void controlRestartFlagCallback(const std_msgs::Bool::ConstPtr& msg);
//   void remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg);

//   void checkMessagesForSafety();
//   bool updateState();
//   bool updateControl();
//   void publishState();
//   void publishControl(bool is_valid);
//   void publishHeartbeat();

//   // ROS I/O
//   ros::Subscriber control_mode_sub_;
//   ros::Subscriber body_plan_sub_;
//   ros::Subscriber local_plan_sub_;
//   ros::Subscriber mocap_sub_;
//   ros::Subscriber robot_state_sub_;
//   ros::Subscriber control_restart_flag_sub_;
//   ros::Subscriber remote_heartbeat_sub_;
//   ros::Subscriber single_joint_cmd_sub_;
//   ros::Publisher  robot_state_pub_;
//   ros::Publisher  trajectry_robot_state_pub_;
//   ros::Publisher  robot_heartbeat_pub_;
//   ros::Publisher  leg_command_array_pub_;
//   ros::Publisher  grf_pub_;
//   ros::Publisher  imu_pub_;
//   ros::Publisher  joint_state_pub_;
  

//   // Params / state
//   ros::NodeHandle nh_;
//   bool is_hardware_;
//   std::string controller_id_;
//   std::string estimator_id_;
//   double update_rate_;
//   double publish_rate_;
//   const int num_feet_ = 4;
//   int control_mode_;
//   std::vector<double> torque_limits_;

//   const int SIT = 0, READY = 1, SIT_TO_READY = 2, READY_TO_SIT = 3, SAFETY = 4;
//   const int NONE = 0, LOCAL_PLAN = 1, GRFS = 2;

//   quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;
//   quad_msgs::RobotState last_robot_state_msg_;
//   quad_msgs::GRFArray::ConstPtr last_grf_array_msg_;
//   std_msgs::Header::ConstPtr last_remote_heartbeat_msg_;
//   std_msgs::Header last_robot_heartbeat_msg_;

//   double last_state_time_;
//   double remote_heartbeat_received_time_;

//   const double transition_duration_ = 1.0;

//   double input_timeout_;
//   double state_timeout_;
//   double heartbeat_timeout_;
//   double remote_latency_threshold_warn_;
//   double remote_latency_threshold_error_;

//   quad_msgs::LegCommandArray leg_command_array_msg_;
//   quad_msgs::GRFArray grf_array_msg_;
//   Eigen::VectorXd user_tx_data_;
//   Eigen::VectorXd user_rx_data_;

//   ros::Time transition_timestamp_;
//   std::vector<double> safety_kp_, safety_kd_;
//   std::vector<double> sit_kp_, sit_kd_;
//   std::vector<double> stand_kp_, stand_kd_;
//   std::vector<double> stance_kp_, stance_kd_;
//   std::vector<double> swing_kp_, swing_kd_;
//   std::vector<double> swing_kp_cart_, swing_kd_cart_;
//   std::vector<double> stand_joint_angles_;
//   std::vector<double> sit_joint_angles_;

//   std::shared_ptr<quad_utils::QuadKD> quadKD_;
//   std::shared_ptr<LegController> leg_controller_;
//   std::shared_ptr<StateEstimator> state_estimator_;
//   std::shared_ptr<HardwareInterface> hardware_interface_;

//   geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_;
//   sensor_msgs::Imu last_imu_msg_;
//   sensor_msgs::JointState last_joint_state_msg_;

//   Eigen::Vector3d vel_estimate_;
//   Eigen::Vector3d mocap_vel_estimate_;
//   Eigen::Vector3d imu_vel_estimate_;
//   double filter_time_constant_;
//   double filter_weight_;
//   double mocap_dropout_threshold_;
//   double mocap_rate_;

//   double last_mainboard_time_;
//   ros::Time last_mocap_time_;
//   ros::Time t_pub_;

//   int argc_;
//   char** argv_;
// };

// #endif  // ROBOT_DRIVER_H
