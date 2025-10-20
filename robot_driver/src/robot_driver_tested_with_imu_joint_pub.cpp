// #include "robot_driver/robot_driver.h"

// RobotDriver::RobotDriver(ros::NodeHandle nh, int argc, char **argv)
//     : nh_(nh), argc_(argc), argv_(argv) {
//   // Load parameters
//   std::string robot_name, imu_topic, joint_state_topic, grf_topic,
//       robot_state_topic, trajectory_state_topic, local_plan_topic,
//       leg_command_array_topic, control_mode_topic, remote_heartbeat_topic,
//       robot_heartbeat_topic, single_joint_cmd_topic, mocap_topic,
//       control_restart_flag_topic;

//   quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name, std::string("jamal"));
//   quad_utils::loadROSParam(nh_, "topics/state/imu", imu_topic);
//   quad_utils::loadROSParam(nh_, "topics/state/joints", joint_state_topic);
//   quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
//   quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
//   quad_utils::loadROSParam(nh_, "topics/state/trajectory", trajectory_state_topic);
//   quad_utils::loadROSParam(nh_, "/topics/heartbeat/remote", remote_heartbeat_topic);
//   quad_utils::loadROSParam(nh_, "topics/heartbeat/robot", robot_heartbeat_topic);
//   quad_utils::loadROSParam(nh_, "topics/control/grfs", grf_topic);
//   quad_utils::loadROSParam(nh_, "topics/control/joint_command", leg_command_array_topic);
//   quad_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);
//   quad_utils::loadROSParam(nh_, "topics/control/single_joint_command", single_joint_cmd_topic);
//   quad_utils::loadROSParam(nh_, "topics/control/restart_flag", control_restart_flag_topic);
//   quad_utils::loadROSParam(nh_, "topics/mocap", mocap_topic);

//   quad_utils::loadROSParamDefault(nh_, "robot_driver/is_hardware", is_hardware_, true);
//   quad_utils::loadROSParamDefault(nh_, "robot_driver/controller", controller_id_, std::string("inverse_dynamics"));
//   quad_utils::loadROSParamDefault(nh_, "robot_driver/estimator", estimator_id_, std::string("unitree_estimator"));
//   quad_utils::loadROSParam(nh_, "/robot_driver/update_rate", update_rate_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/publish_rate", publish_rate_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/mocap_rate", mocap_rate_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/mocap_dropout_threshold", mocap_dropout_threshold_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/filter_time_constant", filter_time_constant_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/input_timeout", input_timeout_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/state_timeout", state_timeout_);
//   quad_utils::loadROSParam(nh_, "/robot_driver/heartbeat_timeout", heartbeat_timeout_);
//   quad_utils::loadROSParam(nh_, "robot_driver/sit_kp", sit_kp_);
//   quad_utils::loadROSParam(nh_, "robot_driver/sit_kd", sit_kd_);
//   quad_utils::loadROSParam(nh_, "robot_driver/stand_kp", stand_kp_);
//   quad_utils::loadROSParam(nh_, "robot_driver/stand_kd", stand_kd_);
//   quad_utils::loadROSParam(nh_, "robot_driver/stance_kp", stance_kp_);
//   quad_utils::loadROSParam(nh_, "robot_driver/stance_kd", stance_kd_);
//   quad_utils::loadROSParam(nh_, "robot_driver/swing_kp", swing_kp_);
//   quad_utils::loadROSParam(nh_, "robot_driver/swing_kd", swing_kd_);
//   quad_utils::loadROSParam(nh_, "robot_driver/swing_kp_cart", swing_kp_cart_);
//   quad_utils::loadROSParam(nh_, "robot_driver/swing_kd_cart", swing_kd_cart_);
//   quad_utils::loadROSParam(nh_, "robot_driver/safety_kp", safety_kp_);
//   quad_utils::loadROSParam(nh_, "robot_driver/safety_kd", safety_kd_);
//   quad_utils::loadROSParam(nh_, "robot_driver/stand_joint_angles", stand_joint_angles_);
//   quad_utils::loadROSParam(nh_, "robot_driver/sit_joint_angles", sit_joint_angles_);
//   quad_utils::loadROSParam(nh_, "robot_driver/torque_limit", torque_limits_);

//   if (torque_limits_.size() != 3) {
//     ROS_FATAL("robot_driver/torque_limit must contain exactly 3 elements (per joint in leg).");
//     throw std::runtime_error("Invalid torque_limit size");
//   }

//   // Pubs/subs
//   local_plan_sub_ = nh_.subscribe(local_plan_topic, 1, &RobotDriver::localPlanCallback, this,
//                                   ros::TransportHints().tcpNoDelay(true));
//   control_mode_sub_ = nh_.subscribe(control_mode_topic, 1, &RobotDriver::controlModeCallback, this);
//   single_joint_cmd_sub_ = nh_.subscribe(single_joint_cmd_topic, 1, &RobotDriver::singleJointCommandCallback, this);
//   remote_heartbeat_sub_ = nh_.subscribe(remote_heartbeat_topic, 1, &RobotDriver::remoteHeartbeatCallback, this);
//   control_restart_flag_sub_ = nh_.subscribe(control_restart_flag_topic, 1, &RobotDriver::controlRestartFlagCallback, this);

//   grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic, 1);
//   leg_command_array_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic, 1);
//   robot_heartbeat_pub_ = nh_.advertise<std_msgs::Header>(robot_heartbeat_topic, 1);
//   trajectry_robot_state_pub_ = nh_.advertise<quad_msgs::RobotState>(trajectory_state_topic, 1);

//   if (is_hardware_) {
//     ROS_INFO("Loading hardware robot driver");
//     mocap_sub_ = nh_.subscribe(mocap_topic, 1000, &RobotDriver::mocapCallback, this,
//                                ros::TransportHints().tcpNoDelay(true));
//     robot_state_pub_ = nh_.advertise<quad_msgs::RobotState>(robot_state_topic, 1);
//     imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1);
//     joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
//   } else {
//     ROS_INFO("Loading sim robot driver");
//     robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &RobotDriver::robotStateCallback, this,
//                                      ros::TransportHints().tcpNoDelay(true));
//   }

//   // Kinematics
//   quadKD_ = std::make_shared<quad_utils::QuadKD>();

//   // Hardware interface (Jamal)
//   if (is_hardware_) {
//     hardware_interface_ = std::make_shared<JamalInterface>();
//     ROS_INFO("Jamal interface constructed");
//     hardware_interface_->init(nh_, nh_);   // IMPORTANT: initialize pubs/subs inside the interface
//   }

//   // Leg controller
//   initLegController();

//   // Initial state
//   control_mode_ = SIT;
//   remote_heartbeat_received_time_ = std::numeric_limits<double>::max();
//   last_state_time_ = std::numeric_limits<double>::max();

//   last_robot_state_msg_.header.stamp = ros::Time::now();
//   t_pub_ = ros::Time::now();

//   // State/control structures
//   double dt = 1.0 / std::max(1.0, mocap_rate_);
//   filter_weight_ = 1.0 - dt / std::max(1e-3, filter_time_constant_);
//   initStateControlStructs();

//   // Estimator
//   initStateEstimator();

//   ROS_INFO("RobotDriver initialization completed");
// }

// void RobotDriver::initStateEstimator() {
//   if (estimator_id_ == "comp_filter") {
//     state_estimator_ = std::make_shared<CompFilterEstimator>();
//   } else if (estimator_id_ == "ekf_filter") {
//     state_estimator_ = std::make_shared<EKFEstimator>();
//   } else if (estimator_id_ == "unitree_estimator") {
//     state_estimator_ = std::make_shared<UnitreeEstimator>();
//   } else {
//     ROS_ERROR_STREAM("Invalid estimator id '" << estimator_id_ << "', returning nullptr");
//     state_estimator_ = nullptr;
//   }
//   if (state_estimator_) state_estimator_->init(nh_);
// }

// void RobotDriver::initLegController() {
//   if (controller_id_ == "inverse_dynamics") {
//     leg_controller_ = std::make_shared<InverseDynamicsController>();
//   } else if (controller_id_ == "grf_pid") {
//     leg_controller_ = std::make_shared<GrfPidController>();
//   } else if (controller_id_ == "joint") {
//     leg_controller_ = std::make_shared<JointController>();
//   } else {
//     ROS_ERROR_STREAM("Invalid controller id '" << controller_id_ << "', returning nullptr");
//     leg_controller_ = nullptr;
//   }
//   if (leg_controller_) {
//     leg_controller_->init(stance_kp_, stance_kd_, swing_kp_, swing_kd_, swing_kp_cart_, swing_kd_cart_);
//   }
// }

// void RobotDriver::initStateControlStructs() {
//   vel_estimate_.setZero();
//   mocap_vel_estimate_.setZero();
//   imu_vel_estimate_.setZero();

//   last_joint_state_msg_.name.resize(12);
//   last_joint_state_msg_.position.resize(12);
//   last_joint_state_msg_.velocity.resize(12);
//   last_joint_state_msg_.effort.resize(12);

//   grf_array_msg_.vectors.resize(4);
//   grf_array_msg_.points.resize(4);
//   grf_array_msg_.contact_states.resize(4);
//   grf_array_msg_.header.frame_id = "map";

//   user_tx_data_.resize(1);
// }

// void RobotDriver::controlModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
//   if ((control_mode_ == SIT_TO_READY) || (control_mode_ == READY_TO_SIT)) return;

//   if ((msg->data == READY) && (control_mode_ == SIT)) {
//     control_mode_ = SIT_TO_READY;
//     transition_timestamp_ = ros::Time::now();
//   } else if ((msg->data == SIT) && (control_mode_ == READY)) {
//     control_mode_ = READY_TO_SIT;
//     transition_timestamp_ = ros::Time::now();
//   } else if (msg->data == SIT || (msg->data == SAFETY)) {
//     control_mode_ = msg->data;
//   }
// }

// void RobotDriver::singleJointCommandCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
//   if (auto *c = dynamic_cast<JointController *>(leg_controller_.get())) {
//     c->updateSingleJointCommand(msg);
//   }
// }

// void RobotDriver::controlRestartFlagCallback(const std_msgs::Bool::ConstPtr &msg) {
//   user_tx_data_[0] = (msg->data) ? 1 : 0;
// }

// void RobotDriver::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg) {
//   last_local_plan_msg_ = msg;

//   ros::Time t_now = ros::Time::now();
//   double round_trip_time_diff = (t_now - last_local_plan_msg_->state_timestamp).toSec();

//   if (leg_controller_) {
//     leg_controller_->updateLocalPlanMsg(last_local_plan_msg_, t_now);
//   }
// }

// void RobotDriver::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//   Eigen::Vector3d pos;
//   quad_utils::pointMsgToEigen(msg->pose.position, pos);

//   ros::Time t_now = ros::Time::now();

//   if (!last_mocap_msg_) {
//     last_mocap_msg_ = msg;
//     last_mocap_time_ = t_now;
//     return;
//   }

//   double t_diff_mocap_msg = (msg->header.stamp - last_mocap_msg_->header.stamp).toSec();
//   double t_mocap_ros_latency = (t_now - msg->header.stamp).toSec();
//   (void)t_mocap_ros_latency; // currently unused

//   last_mocap_time_ = t_now;

//   if (std::abs(t_diff_mocap_msg - 1.0 / std::max(1.0, mocap_rate_)) < mocap_dropout_threshold_) {
//     if (auto* c = dynamic_cast<CompFilterEstimator *>(state_estimator_.get())) {
//       c->mocapCallBackHelper(msg, pos);
//     }
//   } else {
//     ROS_WARN_THROTTLE(0.1, "Mocap time diff exceeds threshold, holding last value");
//   }

//   last_mocap_msg_ = msg;
// }

// void RobotDriver::robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg) {
//   last_robot_state_msg_ = *msg;
// }

// void RobotDriver::remoteHeartbeatCallback(const std_msgs::Header::ConstPtr &msg) {
//   double remote_heartbeat_sent_time = msg->stamp.toSec();
//   remote_heartbeat_received_time_ = ros::Time::now().toSec();
//   double t_latency = remote_heartbeat_received_time_ - remote_heartbeat_sent_time;
//   (void)t_latency; // reserved for diagnostics
// }

// void RobotDriver::checkMessagesForSafety() {
//   if (control_mode_ == SAFETY) return;

//   if (std::abs(ros::Time::now().toSec() - remote_heartbeat_received_time_) >= heartbeat_timeout_ &&
//       remote_heartbeat_received_time_ != std::numeric_limits<double>::max()) {
//     control_mode_ = SAFETY;
//     ROS_WARN_THROTTLE(1, "Remote heartbeat late/lost, entering SAFETY");
//   }

//   if (!is_hardware_ &&
//       std::abs(ros::Time::now().toSec() - last_state_time_) >= state_timeout_ &&
//       last_state_time_ != std::numeric_limits<double>::max()) {
//     control_mode_ = SAFETY;
//     transition_timestamp_ = ros::Time::now();
//     ROS_WARN_THROTTLE(1, "State messages lost, entering SAFETY");
//   }
// }

// bool RobotDriver::updateState() {
//   if (is_hardware_) {
//     ros::Time t_now = ros::Time::now();
//     ros::Duration period = ros::Duration(1.0 / std::max(1.0, update_rate_));
//     hardware_interface_->recv(last_joint_state_msg_, last_imu_msg_, t_now, period);

//     if (state_estimator_) {
//       state_estimator_->loadSensorMsg(last_imu_msg_, last_joint_state_msg_);
//     } else {
//       ROS_WARN_THROTTLE(1, "No state estimator initialized");
//       return false;
//     }

//     if (last_mocap_msg_) state_estimator_->loadMocapMsg(last_mocap_msg_);

//     return state_estimator_->updateOnce(last_robot_state_msg_);
//   } else {
//     return true; // sim provides via subscribers
//   }
// }

// void RobotDriver::publishState() {
//   if (is_hardware_) {
//     imu_pub_.publish(last_imu_msg_);
//     joint_state_pub_.publish(last_joint_state_msg_);
//     robot_state_pub_.publish(last_robot_state_msg_);
//   }
// }

// bool RobotDriver::updateControl() {
//   bool valid_cmd = true;

//   if (leg_controller_ && leg_controller_->overrideStateMachine()) {
//     return leg_controller_->computeLegCommandArray(last_robot_state_msg_, leg_command_array_msg_, grf_array_msg_);
//   }

//   checkMessagesForSafety();

//   if (last_robot_state_msg_.header.stamp.toSec() == 0) return false;

//   Eigen::VectorXd joint_positions(3 * num_feet_), joint_velocities(3 * num_feet_);
//   quad_utils::vectorToEigen(last_robot_state_msg_.joints.position, joint_positions);
//   quad_utils::vectorToEigen(last_robot_state_msg_.joints.velocity, joint_velocities);

//   leg_command_array_msg_.leg_commands.resize(num_feet_);

//   if (control_mode_ == SAFETY) {
//     for (int i = 0; i < num_feet_; ++i) {
//       auto& lc = leg_command_array_msg_.leg_commands.at(i);
//       lc.motor_commands.resize(3);
//       for (int j = 0; j < 3; ++j) {
//         robot_driver_utils::loadMotorCommandMsg(0, 0, 0, safety_kp_.at(j), safety_kd_.at(j), lc.motor_commands.at(j));
//       }
//     }
//   } else if (control_mode_ == SIT) {
//     for (int i = 0; i < num_feet_; ++i) {
//       auto& lc = leg_command_array_msg_.leg_commands.at(i);
//       lc.motor_commands.resize(3);
//       for (int j = 0; j < 3; ++j) {
//         robot_driver_utils::loadMotorCommandMsg(sit_joint_angles_.at(j), 0, 0, sit_kp_.at(j), sit_kd_.at(j), lc.motor_commands.at(j));
//       }
//     }
//   } else if (control_mode_ == READY) {
//     if (!leg_controller_->computeLegCommandArray(last_robot_state_msg_, leg_command_array_msg_, grf_array_msg_)) {
//       for (int i = 0; i < num_feet_; ++i) {
//         auto& lc = leg_command_array_msg_.leg_commands.at(i);
//         lc.motor_commands.resize(3);
//         for (int j = 0; j < 3; ++j) {
//           robot_driver_utils::loadMotorCommandMsg(stand_joint_angles_.at(j), 0, 0, stand_kp_.at(j), stand_kd_.at(j), lc.motor_commands.at(j));
//         }
//       }
//     }
//   } else if (control_mode_ == SIT_TO_READY || control_mode_ == READY_TO_SIT) {
//     ros::Duration duration = ros::Time::now() - transition_timestamp_;
//     double t_interp = duration.toSec() / transition_duration_;
//     if (t_interp >= 1.0) {
//       control_mode_ = (control_mode_ == SIT_TO_READY) ? READY : SIT;
//       return valid_cmd;
//     }
//     for (int i = 0; i < num_feet_; ++i) {
//       auto& lc = leg_command_array_msg_.leg_commands.at(i);
//       lc.motor_commands.resize(3);
//       for (int j = 0; j < 3; ++j) {
//         double src = (control_mode_ == SIT_TO_READY) ? sit_joint_angles_.at(j) : stand_joint_angles_.at(j);
//         double dst = (control_mode_ == SIT_TO_READY) ? stand_joint_angles_.at(j) : sit_joint_angles_.at(j);
//         double ang = (dst - src) * t_interp + src;
//         robot_driver_utils::loadMotorCommandMsg(ang, 0, 0, stand_kp_.at(j), stand_kd_.at(j), lc.motor_commands.at(j));
//       }
//     }
//   } else {
//     ROS_WARN_THROTTLE(0.5, "Invalid control mode; updateControl() aborted");
//     return false;
//   }

//   const int knee_idx = 2;
//   const double knee_soft_ub = 3.0;
//   const double knee_soft_ub_kd = 50.0;

//   for (int i = 0; i < num_feet_; ++i) {
//     for (int j = 0; j < 3; ++j) {
//       int joint_idx = 3 * i + j;

//       if (j == knee_idx && joint_positions(joint_idx) > knee_soft_ub) {
//         auto& mc = leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
//         mc.torque_ff = std::max(mc.torque_ff - knee_soft_ub_kd * (joint_positions(joint_idx) - knee_soft_ub),
//                                 -torque_limits_[j]);
//       }

//       auto& cmd = leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
//       double pos_component = cmd.kp * (cmd.pos_setpoint - joint_positions[joint_idx]);
//       double vel_component = cmd.kd * (cmd.vel_setpoint - joint_velocities[joint_idx]);
//       double fb_component  = pos_component + vel_component;
//       double effort        = fb_component + cmd.torque_ff;
//       double fb_ratio      = std::abs(fb_component) / (std::abs(fb_component) + std::abs(cmd.torque_ff) + 1e-9);

//       if (std::abs(cmd.torque_ff) >= torque_limits_[j]) {
//         ROS_WARN("Leg %d motor %d: ff=%.3f exceeds limit %.3f", i, j, cmd.torque_ff, torque_limits_[j]);
//       }
//       if (std::abs(effort) >= torque_limits_[j]) {
//         ROS_WARN("Leg %d motor %d: total effort=%.3f exceeds limit %.3f", i, j, effort, torque_limits_[j]);
//         effort = std::min(std::max(effort, -torque_limits_[j]), torque_limits_[j]);
//       }

//       cmd.pos_component = pos_component;
//       cmd.vel_component = vel_component;
//       cmd.fb_component  = fb_component;
//       cmd.effort       = effort;
//       cmd.fb_ratio     = fb_ratio;
//     }
//   }

//   return valid_cmd;
// }

// void RobotDriver::publishControl(bool is_valid) {
//   leg_command_array_msg_.header.stamp = ros::Time::now();
//   leg_command_array_pub_.publish(leg_command_array_msg_);
//   grf_array_msg_.header.stamp = leg_command_array_msg_.header.stamp;
//   grf_pub_.publish(grf_array_msg_);

//   if (is_hardware_ && is_valid) {
//     ros::Time t_start = ros::Time::now();
//     ros::Duration period = ros::Duration(1.0 / std::max(1.0, update_rate_));
//     hardware_interface_->send(leg_command_array_msg_, t_start, period);
//     ros::Time t_end = ros::Time::now();
//     ROS_INFO_THROTTLE(1.0, "t_diff_mb_send = %6.4f", (t_end - t_start).toSec());
//   }
// }

// void RobotDriver::publishHeartbeat() {
//   if ((ros::Time::now() - last_robot_heartbeat_msg_.stamp).toSec() >= 1.0 / std::max(1.0, publish_rate_)) {
//     last_robot_heartbeat_msg_.stamp = ros::Time::now();
//     robot_heartbeat_pub_.publish(last_robot_heartbeat_msg_);
//   }
// }

// void RobotDriver::spin() {
//   ros::Rate r(update_rate_);

//   while (ros::ok()) {
//     ros::spinOnce();
//     updateState();
//     bool is_valid = updateControl();
//     publishControl(is_valid);
//     publishState();
//     publishHeartbeat();
//     r.sleep();
//   }
// }
