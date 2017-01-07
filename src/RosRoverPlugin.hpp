// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017
// Basically this plugin handles ROS topics communication

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "RoverModel.hpp"

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>


namespace gazebo{
    class RosRoverPlugin : public ModelPlugin{
        public:
        RosRoverPlugin();
        ~RosRoverPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
        void onUpdate( const common::UpdateInfo &info );

        event::ConnectionPtr updateConnection;
        boost::shared_ptr<ros::AsyncSpinner> async_ros_spin;

        void actionCallback(const std_msgs::Int32::ConstPtr& actionMsg);
        const std_msgs::Float32 getReward() const;
        const geometry_msgs::Point32 getPositionState() const;
        const geometry_msgs::Quaternion getOrientationState() const;
        const std_msgs::Int32 getVelocityState() const;
        const std_msgs::Int32 getSteeringState() const;

        ros::Subscriber actionSubscriber;
        ros::Publisher rewardPublisher;
        ros::Publisher positionStatePublisher, orientationStatePublisher;
        ros::Publisher velocityStatePublisher, steeringStatePublisher;

        boost::shared_ptr<RoverModel> roverModel;
    };
    GZ_REGISTER_MODEL_PLUGIN(RosRoverPlugin)
}
