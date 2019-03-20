#include<gazebo_plugins/gazebo_ros_utils.h>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>
#include<ignition/math/Vector3.hh>

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
         //GazeboRosPtr gazebo_ros_;
   };
   
   // Constructor
   AckermanSteer::AckermanSteer() {}

   // Destructor
   AckermanSteer::~AckermanSteer() {}

   // Required Load method:
   void AckermanSteer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
   {
      //store the pointer to the model
      this->model = _model;
      //gazebo_ros_ = GazeboRosPtr( new GazeboRos(_model, _sdf, "AckermanSteer") );
             
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

