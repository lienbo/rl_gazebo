// Copyright © 2016 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// September 2016

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include "carina_plugin.pb.h"

namespace gazebo{
    typedef const boost::shared_ptr<const carina_plugin::msgs::Throttle> ThrottlePtr;

    class CarinaPlugin : public ModelPlugin{
        public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdfFile);
        void OnUpdate(const common::UpdateInfo & info);
        void ThrottleCallback(ThrottlePtr &throttleMsg);
    
        private:
        physics::ModelPtr carinaModel;
        event::ConnectionPtr updateConnection;
        transport::NodePtr node;
        transport::SubscriberPtr throttleSubscriber;
    };
    GZ_REGISTER_MODEL_PLUGIN(CarinaPlugin)
}
