// // #include "robot_driver/hardware_interfaces/jamal_interface.h"
// // #include <std_msgs/Float64MultiArray.h>
// // #include <std_msgs/Int16MultiArray.h>
// // #include <trajectory_msgs/JointTrajectory.h>
// // #include <trajectory_msgs/JointTrajectoryPoint.h>

// // // JamalInterface::JamalInterface(ros::NodeHandle nh)
// // //   : HardwareInterface(nh)  // Call base class constructor
// // // {
// // // }
// // JamalInterface::JamalInterface() {}
// // // void JamalInterface::loadInterface(int argc, char** argv) {
// // //   // Initialize publishers, subscribers, etc.
// // //   ROS_INFO("JamalInterface loaded.");
// // // }

// // // void JamalInterface::unloadInterface() {
// // //   // Clean up resources if needed
// // //   ROS_INFO("JamalInterface unloaded.");
// // // }
// // // void JamalInterface::loadInterface(int argc, char** argv) {
// // //   /// Ghost MBLink interface class
// // //   mblink_.start(argc, argv);
// // //   mblink_.rxstart();
// // //   mblink_.setRetry("_UPST_ADDRESS", 255);
// // //   mblink_.setRetry("UPST_LOOP_DELAY", 1);
// // // }

// // // void JamalInterface::unloadInterface() { mblink_.rxstop(); }

// // void JamalInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

// //   // Subscribe to IMU data
// //   imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("/robot_1/trunk_imu", 10, &JamalInterface::imuCallback, this);
// //   joint_state_sub_ = root_nh.subscribe<sensor_msgs::JointState>("/robot_1/joint_states", 10, &JamalInterface::jointStateCallback, this);
// //   command_pub_ = root_nh.advertise<trajectory_msgs::JointTrajectory>("/joint_controller/command", 10);

// //   // return true;
// // }

// // void JamalInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

// //   imu_msg_ = *msg;

// // }

// // void JamalInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
// //   // Resize once to 12
// //   joint_state_msg_.name.resize(12);
// //   joint_state_msg_.position.resize(12);
// //   joint_state_msg_.velocity.resize(12);
// //   joint_state_msg_.effort.resize(12);

// //   for (size_t i = 0; i < msg->name.size(); ++i) {
// //     const std::string& name = msg->name[i];

// //     // for (int j = 0; j < 12; ++j) {
// //       // if (joint_names_[j] == name) {
// //     // joint_state_msg_.name[j] = name;
// //     // joint_state_msg_.position[j] = msg->position[i];
// //     // joint_state_msg_.velocity[j] = msg->velocity[i];
// //     // joint_state_msg_.effort[j] = kt_vec_[j] * msg->effort[i];

// //     joint_state_msg_.name[i] = name;
// //     joint_state_msg_.position[i] = msg->position[i];
// //     joint_state_msg_.velocity[i] = msg->velocity[i];
// //     joint_state_msg_.effort[i] = kt_vec_[i] * msg->effort[i];
// //       // }
// //     // }
// //   }
// // }


// // bool JamalInterface::recv(sensor_msgs::JointState& joint_state_msg,
// //                           sensor_msgs::Imu& imu_msg,
// //                           const ros::Time& time,
// //                           const ros::Duration& /*period*/) {
// //   joint_state_msg = joint_state_msg_;
// //   imu_msg = imu_msg_;

// //   std::ostringstream oss;
// //   oss << "Received Joint States (from /motor_states):\n";
// //   for (int i = 0; i < 12; ++i) {
// //     oss << "Joint " << i << " (" << joint_state_msg.name[i] << "): "
// //         << "pos = " << joint_state_msg.position[i] << ", "
// //         << "vel = " << joint_state_msg.velocity[i] << ", "
// //         << "tau = " << joint_state_msg.effort[i] << "\n";
// //   }
// //   ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
// //   return true;
// // }


// // bool JamalInterface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg, 
// //   const ros::Time& /*time*/, const ros::Duration& /*period*/) {

// //   trajectory_msgs::JointTrajectory traj_msg;
// //   traj_msg.header.stamp = ros::Time::now();

// //   // One trajectory point containing all desired values
// //   trajectory_msgs::JointTrajectoryPoint point;

// //   traj_msg.joint_names = joint_names_;
// //   int leg_command_heartbeat = 1;

// //   int a = 0; /// very important
// //   for (int i = 0; i < 4; ++i) {  // For each leg
// //   // std::cout << "leg = " << i << std::endl;
// //   quad_msgs::LegCommand leg_command =
// //       last_leg_command_array_msg.leg_commands.at(i);

// //   for (int j = 0; j < 3; ++j) {  // For each joint
// //     // std::cout << "joint = " << j << std::endl;
// //     jointData_[a].posDes_ = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
// //     jointData_[a].velDes_ = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
// //     jointData_[a].ff_     = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
// //     a = a+1;
// //   }
// // }
// //   point.positions.clear();
// //   point.velocities.clear();
// //   point.effort.clear();
// //   // Positions 
// //   for (int i = 0; i < 12; ++i)
// //     point.positions.push_back(jointData_[i].posDes_);

// //   // Velocities
// //   for (int i = 0; i < 12; ++i)
// //     point.velocities.push_back(jointData_[i].velDes_);

// //   // Efforts (using ff_ for feedforward torques/forces)
// //   for (int i = 0; i < 12; ++i)
// //     point.effort.push_back(jointData_[i].ff_);

// //   // Set time_from_start (required field)
// //   point.time_from_start = ros::Duration(0.01); // e.g., 10ms

// //   // Add the point to trajectory
// //   traj_msg.points.push_back(point);

// //   // Publish
// //   command_pub_.publish(traj_msg);


// //   ROS_INFO_STREAM_THROTTLE(0.5,
// //     "Publishing Joint Commands (posDes, velDes, ff):\n" <<
// //     [&]() {
// //     std::ostringstream oss;
// //     for (int i = 0; i < 12; ++i) {
// //       oss << "Joint " << i << ": ["
// //           << jointData_[i].posDes_ << ", "
// //           << jointData_[i].velDes_ << ", "
// //           << jointData_[i].ff_ << "]\n";
// //     }
// //     return oss.str();
// //     }()
// //     );
// //     return true;
// // }

// #include "robot_driver/hardware_interfaces/jamal_interface.h"

// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Int16MultiArray.h>
// #include <sstream>

// JamalInterface::JamalInterface() {}

// void JamalInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
//   // Allow overriding topics from params if present
//   root_nh.param<std::string>("topics/state/imu",     imu_topic_,    imu_topic_);
//   root_nh.param<std::string>("topics/state/joints",  joints_topic_, joints_topic_);
//   root_nh.param<std::string>("topics/control/trajectory_command", traj_topic_, traj_topic_);

//   // Wire subs/pubs
//   imu_sub_         = root_nh.subscribe<sensor_msgs::Imu>(imu_topic_, 10, &JamalInterface::imuCallback, this);
//   joint_state_sub_ = root_nh.subscribe<sensor_msgs::JointState>(joints_topic_, 10, &JamalInterface::jointStateCallback, this);
//   command_pub_     = root_nh.advertise<trajectory_msgs::JointTrajectory>(traj_topic_, 10);

//   // Prepare joint_state_msg_ storage
//   joint_state_msg_.name.resize(12);
//   joint_state_msg_.position.assign(12, 0.0);
//   joint_state_msg_.velocity.assign(12, 0.0);
//   joint_state_msg_.effort.assign(12, 0.0);

//   ROS_INFO_STREAM("JamalInterface initialized:"
//                   << " imu='"    << imu_topic_
//                   << "' joints='" << joints_topic_
//                   << "' traj='"   << traj_topic_ << "'");
// }

// void JamalInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//   imu_msg_ = *msg;
// }

// void JamalInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//   // Defensive bounds: don’t overrun our 12-slot cache.
//   const size_t N = std::min<size_t>(12, msg->name.size());

//   // Reset to zeros to avoid stale values where not filled
//   for (int i = 0; i < 12; ++i) {
//     joint_state_msg_.name[i]     = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : "";
//     joint_state_msg_.position[i] = 0.0;
//     joint_state_msg_.velocity[i] = 0.0;
//     joint_state_msg_.effort[i]   = 0.0;
//   }

//   for (size_t i = 0; i < N; ++i) {
//     joint_state_msg_.name[i]     = msg->name[i];
//     if (i < msg->position.size()) joint_state_msg_.position[i] = msg->position[i];
//     if (i < msg->velocity.size()) joint_state_msg_.velocity[i] = msg->velocity[i];
//     double eff_in = (i < msg->effort.size()) ? msg->effort[i] : 0.0; // effort may be empty
//     joint_state_msg_.effort[i]   = kt_vec_[i] * eff_in;
//   }
// }

// bool JamalInterface::recv(sensor_msgs::JointState& joint_state_msg,
//                           sensor_msgs::Imu& imu_msg,
//                           const ros::Time& /*time*/,
//                           const ros::Duration& /*period*/) {
//   joint_state_msg = joint_state_msg_;
//   imu_msg         = imu_msg_;

//   // Bounded log
//   std::ostringstream oss;
//   oss << "Received Joint States:\n";
//   for (int i = 0; i < 12; ++i) {
//     oss << "  [" << i << "] " << joint_state_msg.name[i]
//         << " pos=" << joint_state_msg.position[i]
//         << " vel=" << joint_state_msg.velocity[i]
//         << " tau=" << joint_state_msg.effort[i] << "\n";
//   }
//   ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
//   return true;
// }

// bool JamalInterface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg,
//                           const ros::Time& /*time*/, const ros::Duration& period) {
//   // Build single-point JointTrajectory each cycle
//   trajectory_msgs::JointTrajectory traj_msg;
//   traj_msg.header.stamp = ros::Time::now();
//   traj_msg.joint_names  = joint_names_;   // must match controller joint order

//   trajectory_msgs::JointTrajectoryPoint point;
//   point.positions.reserve(12);
//   point.velocities.reserve(12);
//   point.effort.reserve(12);

//   // Collect desired commands leg-major (4 legs x 3 joints)
//   int a = 0;
//   for (int leg = 0; leg < 4; ++leg) {
//     const auto& leg_cmd = last_leg_command_array_msg.leg_commands.at(leg);
//     for (int j = 0; j < 3; ++j, ++a) {
//       jointData_[a].posDes_ = leg_cmd.motor_commands.at(j).pos_setpoint;
//       jointData_[a].velDes_ = leg_cmd.motor_commands.at(j).vel_setpoint;
//       jointData_[a].ff_     = leg_cmd.motor_commands.at(j).torque_ff;
//     }
//   }

//   for (int i = 0; i < 12; ++i) point.positions.push_back(jointData_[i].posDes_);
//   for (int i = 0; i < 12; ++i) point.velocities.push_back(jointData_[i].velDes_);
//   for (int i = 0; i < 12; ++i) point.effort.push_back(jointData_[i].ff_);

//   // Make time_from_start strictly increasing (avoid “non-monotonic” drops)
//   ros::Duration step = period.isZero() ? ros::Duration(0.01) : period;
//   traj_time_from_start_ += step;
//   point.time_from_start = traj_time_from_start_;

//   traj_msg.points.push_back(point);
//   command_pub_.publish(traj_msg);

//   ROS_INFO_THROTTLE(0.5, "Published JointTrajectory (time_from_start=%.3f s)",
//                     traj_time_from_start_.toSec());
//   return true;
// }
