// Copyright © 2016 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// September 2016

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

namespace gazebo{
    class CarinaPlugin : public ModelPlugin{
        public:
        CarinaPlugin();
        ~CarinaPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
        void onUpdate( const common::UpdateInfo &info );
        void actionCallback(const std_msgs::Int32::ConstPtr& actionMsg);
        void steeringCallback(const std_msgs::Float32::ConstPtr& steeringMsg);

        void loadParameters();
        void checkParameterName( const std::string &parameterName );
        void steeringWheelController();
        void applyThrottle(const int &action);
        const std_msgs::Float32 getReward() const;
        const std_msgs::Float32 getSensation() const;

        event::ConnectionPtr updateConnection;
        boost::shared_ptr<ros::AsyncSpinner> async_ros_spin;

        sdf::ElementPtr sdfFile;
        physics::LinkPtr chassisLink;
        physics::ModelPtr carinaModel;
        physics::JointPtr frontLeftJoint, frontRightJoint;

        ros::Subscriber actionSubscriber;
        ros::Subscriber steeringSubscriber;
        ros::Publisher rewardPublisher;
        ros::Publisher sensationPublisher;

        // Angles are in radians. Positive is counterclockwise
        float steeringAngle;
        float vehicleVelocity;
    };
    GZ_REGISTER_MODEL_PLUGIN(CarinaPlugin)
}
