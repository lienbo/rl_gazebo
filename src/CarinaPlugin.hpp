// Copyright © 2016 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// September 2016

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

namespace gazebo{
    class CarinaPlugin : public ModelPlugin{
        public:
        CarinaPlugin();
        ~CarinaPlugin();
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
        void onUpdate( const common::UpdateInfo &info );
        void actionCallback(const std_msgs::String::ConstPtr& actionMsg);
        void applyThrottle(const int& action);
        void steeringCallback(const std_msgs::Float32::ConstPtr& steeringMsg);

        private:
        void loadParameters();
        void checkParameterName( const std::string &parameterName );
        void steeringWheelController();

        event::ConnectionPtr updateConnection;

        sdf::ElementPtr sdfFile;
        physics::LinkPtr chassisLink;
        physics::ModelPtr carinaModel;
        physics::JointPtr frontLeftJoint, frontRightJoint;

        boost::shared_ptr<ros::AsyncSpinner> async_ros_spin;
        ros::Subscriber actionSubscriber;
        ros::Subscriber steeringSubscriber;

        // Angles are in radians. Positive is counterclockwise
        double steeringAngle;
    };
    GZ_REGISTER_MODEL_PLUGIN(CarinaPlugin)
}
