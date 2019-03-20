#include<functional>
#include<gazebo/gazebo.hh>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>
#include<ignition/math/Vector3.hh>

namespace gazebo
{
   class ModelPush : public ModelPlugin
   {
      public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
         //store the pointer to the model
         this->model= _parent;

         // Listen to the update event. This event is broadcast every 
         // simulation iteration.
         this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               std::bind(&ModelPush::OnUpdate, this));
      }

   //Called by the world update start event
   public: void OnUpdate()
   {
      // Apply a small linear velocity to the model
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
   }

   // Pointer to the model
   private: physics::ModelPtr model;

   // Pointer to the update event connection
   private: event::ConnectionPtr updateConnection;

   };

   //Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

