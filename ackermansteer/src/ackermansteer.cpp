#include "ackermansteer.hh"

/*
*  Copyright 2019 Forrest Edwards
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights 
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
* copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all 
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. * IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

namespace gazebo {

  // Constructor
  AckermanSteer::AckermanSteer() {}

  // Destructor
  AckermanSteer::~AckermanSteer() {}

  // Required Load method:
  void AckermanSteer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the Model
    this->model = _parent;
    // Create a new GazeboRos instance
    gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "AckermanSteer"));

    // Check to see if ros is initialized.Prints ROS_FATAL and return if not.
    gazebo_ros_->isInitialized();

    // Function template call to getParameter. Retrieves params passed in _sdf
    gazebo_ros_->getParameterBoolean
       (debug_, "debug", false);
    gazebo_ros_->getParameter<std::string>
       (drive_joint_names_[FL], "FL_driveJoint", "front_left_wheel_bearing");
    gazebo_ros_->getParameter<std::string>
       (drive_joint_names_[FR], "FR_driveJoint", "front_right_wheel_bearing");
    gazebo_ros_->getParameter<std::string>
       (drive_joint_names_[RL], "RL_driveJoint", "rear_left_wheel_bearing");
    gazebo_ros_->getParameter<std::string>
       (drive_joint_names_[RR], "RR_driveJoint", "rear_right_wheel_bearing");
    gazebo_ros_->getParameter<std::string>
       (steer_joint_names_[FL], "FL_steerJoint", "front_left_steer_bearing");
    gazebo_ros_->getParameter<std::string>
       (steer_joint_names_[FR], "FR_steerJoint", "front_right_steer_bearing");
    gazebo_ros_->getParameter<std::string>
       (steer_joint_names_[RL], "RL_steerJoint", "rear_left_steer_bearing");
    gazebo_ros_->getParameter<std::string>
       (steer_joint_names_[RR], "RR_steerJoint", "rear_right_steer_bearing");
    gazebo_ros_->getParameter<std::string>
       (command_topic_, "commandTopic", "cmd_vel");
    gazebo_ros_->getParameter<std::string>
       (odometry_topic_, "odometryTopic", "odom");
    gazebo_ros_->getParameter<std::string>
       (odometry_frame_, "odometryFrame", "odom");
    gazebo_ros_->getParameter<std::string>
       (robot_base_frame_, "robotBaseFrame", "base_footprint");
    gazebo_ros_->getParameterBoolean
       (publishWheelTF_, "publishWheelTF", false);
    gazebo_ros_->getParameterBoolean
       (publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean
       (publishWheelJointState_, "publishWheelJointState", false);
    gazebo_ros_->getParameter<double>
       (wheel_separation_, "wheelSeparation", 0.34);
    gazebo_ros_->getParameter<double>
       (wheelbase_, "wheelbase", 0.5);
    gazebo_ros_->getParameter<double>
       (wheel_diameter_, "wheelDiameter", 0.15);
    gazebo_ros_->getParameter<double>
       (wheel_accel_, "wheelAcceleration", 0.0);
    gazebo_ros_->getParameter<double>
       (wheel_torque_, "wheelTorque", 5.0);
    gazebo_ros_->getParameter<double>
       (update_rate_, "updateRate", 100.0);
    gazebo_ros_->getParameter<double>
       (steer_p_, "steer_p", 1.0);
    gazebo_ros_->getParameter<double>
       (steer_i_, "steer_i", 0.0);
    gazebo_ros_->getParameter<double>
       (steer_d_, "steer_d", 0.0);
    gazebo_ros_->getParameter<double>
       (steer_imax_, "steer_imax", 1.0);
    gazebo_ros_->getParameter<double>
       (steer_imin_, "steer_imin", 1.0);
    gazebo_ros_->getParameter<double>
       (steer_cmd_max_, "steer_max_effort", 20.0);
    gazebo_ros_->getParameter<double>
       (steer_init_angle_, "steer_init_angle", 0.0);
    gazebo_ros_->getParameter<double>
       (drive_p_, "drive_p", 1.0);
    gazebo_ros_->getParameter<double>
       (drive_i_, "drive_i", 0.0);
    gazebo_ros_->getParameter<double>
       (drive_d_, "drive_d", 0.0);
    gazebo_ros_->getParameter<double>
       (drive_imax_, "drive_imax", 1.0);
    gazebo_ros_->getParameter<double>
       (drive_imin_, "drive_imin", 1.0);
    gazebo_ros_->getParameter<double>
       (drive_cmd_max_, "drive_max_effort", 1.0);
    gazebo_ros_->getParameter<double>
       (drive_init_velocity_, "drive_init_velocity", 0.0);


    // create dictionary with string keys, and OdomSource values.
    // Can be either ENCODER(0) or WORLD(1)
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource>
       (odom_source_, "odometrySource", odomOptions, WORLD);

    // Joints & PID's
    steer_joints_.resize(4);
    drive_joints_.resize(4);
    steer_PIDs_.resize(4);
    drive_PIDs_.resize(4);
    for (int i = 0; i < 4; i++) {
      steer_joints_[i] = model->GetJoint(steer_joint_names_[i]);
      drive_joints_[i] = model->GetJoint(drive_joint_names_[i]);
      steer_PIDs_[i].Init(steer_p_, steer_i_, steer_d_, steer_imax_,
            steer_imin_, steer_cmd_max_, -steer_cmd_max_);
      drive_PIDs_[i].Init(drive_p_, drive_i_, drive_d_, drive_imax_,
            drive_imin_, drive_cmd_max_, -drive_cmd_max_);
      switch (i) {
        case FL:
        case FR:
          steer_target_angles_.push_back(steer_init_angle_);
          drive_target_velocities_.push_back(0.0);
          break;
        case RL:
        case RR:
          steer_target_angles_.push_back(0.0);
          drive_target_velocities_.push_back(drive_init_velocity_);
      }
    }

    // Initialize update Rate logic
    if (update_rate_ > 0.0) {
      update_period_ = 1.0/update_rate_;
    } else {
      update_period_ = 1.0;
    }

    // ROS PUB-SUB
    ROS_INFO_NAMED("AckermanSteer", "%s: Try to subuscribe to %s",
          gazebo_ros_->info(), command_topic_.c_str());
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
             boost::bind(&AckermanSteer::cmdVelCallback, this, _1),
             ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("AckermanSteer", "%s: Subscribe to %s",
          gazebo_ros_->info(), command_topic_.c_str());

    this->callback_queue_thread_ =
       boost::thread(boost::bind(&AckermanSteer::QueueThread, this));


    last_update_time_ = this->GazeboTime();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AckermanSteer::OnUpdate, this));
  }

  common::Time AckermanSteer::GazeboTime() {
#if GAZEBO_MAJOR_VERSION >= 8
    return model->GetWorld()->SimTime();
#else
    return model->GetWorld()->GetSimTime();
#endif
  }

  std::vector<double> AckermanSteer::GetAckAngles(double phi) {
    std::vector<double> phi_angles;
    double numerator = 2.0 * wheelbase_ * sin(phi);
    phi_angles.assign(4, 0.0);
    phi_angles[FL] = atan2(numerator,
       (2.0*wheelbase_*cos(phi) - wheel_separation_*sin(phi)) );
    phi_angles[FR] = atan2(numerator,
       (2.0*wheelbase_*cos(phi) + wheel_separation_*sin(phi)) );
    return phi_angles;
  }

  std::vector<double> AckermanSteer::GetDiffSpeeds(double vel, double phi) {
    std::vector<double> wheel_speeds;
    wheel_speeds.assign(4, 0.0);
    wheel_speeds[RL] = vel * (1.0 - (wheel_separation_ * tan(phi) ) /
       (2.0 * wheelbase_) );
    wheel_speeds[RR] = vel * (1.0 + (wheel_separation_ * tan(phi) ) /
       (2.0 * wheelbase_) );
    return wheel_speeds;
  }

  // Called by the world update start event
  void AckermanSteer::OnUpdate() {
    common::Time current_time = GazeboTime();
    common::Time step_time = current_time - last_update_time_;
    if (step_time > update_period_) {
      double steer_ang_curr, steer_error, steer_cmd_effort;
      double drive_vel_curr, drive_error, drive_cmd_effort;
      std::vector<double> ack_steer_angles = GetAckAngles(rot_);
      std::vector<double> ack_drive_velocities = GetDiffSpeeds(x_, rot_);
      steer_target_angles_[FR] = ack_steer_angles[FR];
      steer_target_angles_[FL] = ack_steer_angles[FL];
      steer_target_angles_[RR] = 0.0;
      steer_target_angles_[RL] = 0.0;
      drive_target_velocities_[FR] = 0.0;
      drive_target_velocities_[FL] = 0.0;
      drive_target_velocities_[RR] = ack_drive_velocities[RR];
      drive_target_velocities_[RL] = ack_drive_velocities[RL];

      for (int i = 0; i < 4; i++) {
        switch (i) {
          case FL:
          case FR:
            steer_ang_curr = steer_joints_[i]->GetAngle(X).Radian();
            steer_error = steer_ang_curr - steer_target_angles_[i];
            steer_cmd_effort = steer_PIDs_[i].Update(steer_error, step_time);
            steer_joints_[i]->SetForce(X, steer_cmd_effort);
            break;
          case RL:
          case RR:
            drive_vel_curr = drive_joints_[i]->GetVelocity(Z) *
               wheel_diameter_/2.0;
            drive_error = drive_vel_curr - drive_target_velocities_[i];
            drive_cmd_effort = drive_PIDs_[i].Update(drive_error, step_time);
            drive_joints_[i]->SetForce(Z, drive_cmd_effort);
        }
        if (debug_) {
          double _pe, _ie, _de;
          double pGain = steer_PIDs_[i].GetPGain();
          steer_PIDs_[i].GetErrors(_pe, _ie, _de);
          ROS_INFO("Steer Joints %i", i);
          ROS_INFO("\tCurrent angle: %f \n", steer_ang_curr);
          ROS_INFO("\tTarget angle: %f \n", steer_target_angles_[i]);
          ROS_INFO("\tAngle Error: %f \n", steer_error);
          ROS_INFO("\tEffort: %f \n", steer_cmd_effort);
          ROS_INFO("\tP Gain: %f\n", pGain);
          ROS_INFO("\tP error: %f ", _pe);
          ROS_INFO("\tI error: %f ", _ie);
          ROS_INFO("\tD error: %f ", _de);
        }
        if (debug_) {
          double _pe, _ie, _de;
          double pGain = drive_PIDs_[i].GetPGain();
          drive_PIDs_[i].GetErrors(_pe, _ie, _de);
          ROS_INFO("Drive Joint %i", i);
          ROS_INFO("\tCurrent Vel: %f \n", drive_vel_curr);
          ROS_INFO("\tTarget Vel: %f \n", drive_target_velocities_[i]);
          ROS_INFO("\tVel Error: %f \n", drive_error);
          ROS_INFO("\tEffort: %f \n", drive_cmd_effort);
          ROS_INFO("\tP Gain: %f\n", pGain);
          ROS_INFO("\tP error: %f ", _pe);
          ROS_INFO("\tI error: %f ", _ie);
          ROS_INFO("\tD error: %f ", _de);
        }
      }
      last_update_time_ = current_time;
    }
  }

  void AckermanSteer::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && gazebo_ros_->node()->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void AckermanSteer::cmdVelCallback
     (const geometry_msgs::Twist::ConstPtr& cmd_msg) {
      boost::mutex::scoped_lock scoped_lock(lock);
      x_ = cmd_msg->linear.x;
      rot_ = cmd_msg->angular.z;
  }
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AckermanSteer)

}  // namespace gazebo
