#include<gazebo_plugins/gazebo_ros_utils.h>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>
#include<ignition/math/Vector3.hh>
#include<geometry_msgs/Pose2D.h>
#include<geometry_msgs/Twist.h>
#include<ros/callback_queue.h>
#include<string>
#include<cmath>
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
         common::Time GazeboTime();
         std::vector<double> GetAckAngles(double phi);
         std::vector<double> GetDiffSpeeds(double vel, double phi);
         
         physics::ModelPtr model;                // Pointer to the model
         event::ConnectionPtr updateConnection_;  // Pointer to the update event connection
         GazeboRosPtr gazebo_ros_;
         
         std::string command_topic_;
         std::string odometry_topic_;
         std::string odometry_frame_;
         std::string robot_base_frame_;
         std::string drive_joint_names_[4];
         std::string steer_joint_names_[4]; 
         bool debug_;
         bool publishWheelTF_;
         bool publishOdomTF_;
         bool publishWheelJointState_;
         double wheel_separation_;
         double wheelbase_;
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
         double drive_init_velocity_;
         double drive_cmd_max_;
         double steer_p_;
         double steer_i_;
         double steer_d_;
         double steer_imax_;
         double steer_imin_;
         double steer_max_effort_;
         double steer_init_angle_;
         double steer_cmd_max_;

         double x_;
         double rot_;
         
         ros::Subscriber cmd_vel_subscriber_;
         ros::CallbackQueue queue_;
         boost::thread callback_queue_thread_;
         boost::mutex lock;
         void QueueThread();
         void cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg);
         bool alive_;

         common::Time last_update_time_; 

         OdomSource odom_source_;

         std::vector<physics::JointPtr> steer_joints_, drive_joints_;
         std::vector<common::PID> steer_PIDs_, drive_PIDs_;
         std::vector<double> steer_target_angles_, drive_target_velocities_;
   };
}

