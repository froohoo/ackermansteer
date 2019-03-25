#include<gazebo_plugins/gazebo_ros_utils.h>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>
#include<ignition/math/Vector3.hh>
#include<string>

namespace gazebo
{
   // Wheel order follows cartestion quadrant numbering
   // when x axis indicates primary direction of motion
   enum{ FL, RL, RR, FR };

   class AckermanSteer : public ModelPlugin
   {
      public: 
         AckermanSteer();
         ~AckermanSteer();
         void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
         void OnUpdate();
      private: 
         physics::ModelPtr model;                // Pointer to the model
         event::ConnectionPtr updateConnection;  // Pointer to the update event connection
         GazeboRosPtr gazebo_ros_;
         
         std::string command_topic_;
         std::string odometry_topic_;
         std::string odometry_frame_;
         std::string robot_base_frame_;
         bool publishWheelTF_;
         bool publishOdomTF_;
         bool publishWheelJointState_;
         double wheel_separation_;
         double wheel_diameter_;
         double wheel_accel_;
         double wheel_torque_;
         double update_rate_;

   };
   
   // Constructor
   AckermanSteer::AckermanSteer() {}

   // Destructor
   AckermanSteer::~AckermanSteer() {}

   // Required Load method:
   void AckermanSteer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
   {
      //store the pointer to the Model
      this->model = _model;
      // Create a new GazeboRos instance
      gazebo_ros_ = GazeboRosPtr( new GazeboRos(_model, _sdf, "AckermanSteer") );
      
      //Check to see if ros is initialized.Will just print a ROS_FATAL and return if not.
      gazebo_ros_->isInitialized();
      // Function template call to getParameter. 
      // (T & _value, const char *_tag_name, const T &_default)
      // stores the return in command_topic_
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
      gazebo_ros_->getParameter<double> ( wheel_torque_, "WheelTorgue", 5.0 );
      gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0);
      // Listen to the update event. This event is broadcast every 
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermanSteer::OnUpdate, this));
   }

   //Called by the world update start event
   void AckermanSteer::OnUpdate()
   {
      // Apply a small linear velocity to the model
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
   }


   //Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(AckermanSteer)
}

