#include<gazebo_plugins/gazebo_ros_utils.h>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>
#include<ignition/math/Vector3.hh>
#include<string>

namespace gazebo
{
   // Wheel order follows cartestion quadrant numbering
   // when x axis indicates primary direction of motion

   class AckermanSteer : public ModelPlugin
   {

      enum{ FL, RL, RR, FR };
      enum{X,Y,Z};
      enum OdomSource 
      {
         ENCODER, 
         WORLD
      };
      public: 
         AckermanSteer();
         ~AckermanSteer();
         void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
         void OnUpdate();
      private: 
         common::Time gazeboTime();
         physics::ModelPtr model;                // Pointer to the model
         event::ConnectionPtr updateConnection_;  // Pointer to the update event connection
         GazeboRosPtr gazebo_ros_;
         
         std::string command_topic_;
         std::string odometry_topic_;
         std::string odometry_frame_;
         std::string robot_base_frame_;
         std::string drive_joint_names_[4];
         std::string steer_joint_names_[4]; 
         bool publishWheelTF_;
         bool publishOdomTF_;
         bool publishWheelJointState_;
         double wheel_separation_;
         double wheel_diameter_;
         double wheel_accel_;
         double wheel_torque_;
         double update_rate_;
         double update_period_;

         double drive_p_;
         double drive_i_;
         double drive_d_;
         double drive_imax_;
         double drive_imin_;
         double steer_p_;
         double steer_i_;
         double steer_d_;
         double steer_imax_;
         double steer_imin_;
         double steer_max_effort_;
         double steer_init_angle_;


         common::Time last_update_time_; 

         OdomSource odom_source_;

         std::vector<physics::JointPtr> steer_joints_, drive_joints_;
         std::vector<common::PID> steer_PIDs_, drive_PIDs_;
         std::vector<double> steer_target_angles_;
   };
   
   // Constructor
   AckermanSteer::AckermanSteer() {}

   // Destructor
   AckermanSteer::~AckermanSteer() {}

   // Required Load method:
   void AckermanSteer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
   {
      //store the pointer to the Model
      this->model = _parent;
      // Create a new GazeboRos instance
      gazebo_ros_ = GazeboRosPtr( new GazeboRos(_parent, _sdf, "AckermanSteer") );
      
      //Check to see if ros is initialized.Will just print a ROS_FATAL and return if not.
      gazebo_ros_->isInitialized();
      // Function template call to getParameter, which retrieves params passed in _sdf 
      // (T & _value, const char *_tag_name, const T &_default)
    
      gazebo_ros_->getParameter<std::string> ( drive_joint_names_[FL], "FL_driveJoint", "front_left_wheel_bearing"); 
      gazebo_ros_->getParameter<std::string> ( drive_joint_names_[FR], "FR_driveJoint", "front_right_wheel_bearing");
      gazebo_ros_->getParameter<std::string> ( drive_joint_names_[RL], "RL_driveJoint", "rear_left_wheel_bearing");
      gazebo_ros_->getParameter<std::string> ( drive_joint_names_[RR], "RR_driveJoint", "rear_right_wheel_bearing");
      gazebo_ros_->getParameter<std::string> ( steer_joint_names_[FL], "FL_steerJoint", "front_left_steer_bearing");
      gazebo_ros_->getParameter<std::string> ( steer_joint_names_[FR], "FR_steerJoint", "front_right_steer_bearing");
      gazebo_ros_->getParameter<std::string> ( steer_joint_names_[RL], "RL_steerJoint", "rear_left_steer_bearing");
      gazebo_ros_->getParameter<std::string> ( steer_joint_names_[RR], "RR_steerJoint", "rear_right_steer_bearing");
      gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel");
      gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom");
      gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom");
      gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint");     
      gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
      gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);
      gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false);
      gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.34 );
      gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
      gazebo_ros_->getParameter<double> ( wheel_accel_, "wheelAcceleration", 0.0 );
      gazebo_ros_->getParameter<double> ( wheel_torque_, "wheelTorque", 5.0 );
      gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0);
      gazebo_ros_->getParameter<double> ( steer_p_, "steer_p", 1.0);
      gazebo_ros_->getParameter<double> ( steer_i_, "steer_i", 0.0);
      gazebo_ros_->getParameter<double> ( steer_d_, "steer_d", 0.0);
      gazebo_ros_->getParameter<double> ( steer_imax_, "steer_imax", 1.0);
      gazebo_ros_->getParameter<double> ( steer_imin_, "steer_imin", 1.0);
      gazebo_ros_->getParameter<double> ( steer_max_effort_, "steer_max_effort", 20.0);
      gazebo_ros_->getParameter<double> ( steer_init_angle_, "steer_init_angle", 0.0);
      gazebo_ros_->getParameter<double> ( drive_p_, "drive_p", 1.0);
      gazebo_ros_->getParameter<double> ( drive_i_, "drive_i", 0.0);
      gazebo_ros_->getParameter<double> ( drive_d_, "drive_d", 0.0);
      gazebo_ros_->getParameter<double> ( drive_imax_, "drive_imax", 1.0);
      gazebo_ros_->getParameter<double> ( drive_imin_, "drive_imin", 1.0);


      // create dictionary with string keys, and OdomSource values. 
      // Can be either ENCODER(0) or WORLD(1)
      std::map<std::string, OdomSource> odomOptions;
      odomOptions["encoder"] = ENCODER;
      odomOptions["world"] = WORLD;
      gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD);

      //Joints & PID's
      steer_joints_.resize(4);
      drive_joints_.resize(4);
      steer_PIDs_.resize(4);
      drive_PIDs_.resize(4);
      steer_target_angles_.assign(4,steer_init_angle_);
      for(int i=0; i < 4; i++){
         steer_joints_[i] = model->GetJoint( steer_joint_names_[i]);
         drive_joints_[i] = model->GetJoint( drive_joint_names_[i]);
         steer_PIDs_[i].Init(steer_p_, steer_i_, steer_d_, steer_imax_, steer_imin_);
         drive_PIDs_[i].Init(drive_p_, drive_i_, drive_d_, drive_imax_, drive_imin_);
      }

      


      // Initialize update Rate logic
      if (update_rate_ > 0.0) {
         update_period_ = 1.0/update_rate_;
      } else {
         update_period_ = 1.0;
      }
      last_update_time_ = this->gazeboTime(); 
      // Listen to the update event. This event is broadcast every 
      // simulation iteration.
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermanSteer::OnUpdate, this));
   }
   
   common::Time AckermanSteer::gazeboTime() 
   {
#if GAZEBO_MAJOR_VERSION >= 8
      return model->GetWorld()->SimTime();
#else
      return model->GetWorld()->GetSimTime();
#endif
   }

   //Called by the world update start event
   void AckermanSteer::OnUpdate()
   {
      // Apply a small linear velocity to the model
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      common::Time current_time = gazeboTime();
      common::Time step_time = current_time - last_update_time_;
      if (step_time > update_period_){
         for(int i=0; i<4; i++){
            double steer_angle_curr = steer_joints_[i]->GetAngle(X).Radian();
            double steer_error = steer_angle_curr - steer_target_angles_[i] ;
            double steer_cmd_effort = steer_PIDs_[i].Update(steer_error, step_time);
            if (steer_cmd_effort > steer_max_effort_) steer_cmd_effort = steer_max_effort_;
            if (steer_cmd_effort < -steer_max_effort_) steer_cmd_effort = -steer_max_effort_;
            steer_joints_[i]->SetForce(X, steer_cmd_effort);
            double _pe, _ie, _de;
            double pGain = steer_PIDs_[i].GetPGain();
            steer_PIDs_[i].GetErrors(_pe, _ie, _de);
            if (true){
            ROS_INFO("Steer Joints %i", i); 
            ROS_INFO("\tCurrent angle: %f \n", steer_angle_curr) ;
            ROS_INFO("\tTarget angle: %f \n", steer_target_angles_[i]) ;
            ROS_INFO("\tAngle Error: %f \n", steer_error) ;
            ROS_INFO("\tEffort: %f \n", steer_cmd_effort) ;
            ROS_INFO("\tP Gain: %f\n", pGain);
            ROS_INFO("\tP error: %f ", _pe);
            ROS_INFO("\tI error: %f ", _ie);
            ROS_INFO("\tD error: %f ", _de);
         }

         }

         last_update_time_ = current_time;
      }
   }


   //Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(AckermanSteer)
}

