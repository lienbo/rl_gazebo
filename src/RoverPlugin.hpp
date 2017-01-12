// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include "RoverModel.hpp"
#include "QLearner.hpp"


namespace gazebo{
    class RoverPlugin : public ModelPlugin{
        public:
        RoverPlugin();
        ~RoverPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
        void onUpdate( const common::UpdateInfo &info );
        std::vector<float> getState();
        void printState( const std::vector<float> &observed_state );

        sensors::CameraSensorPtr cameraPtr;
        event::ConnectionPtr updateConnection;
        boost::shared_ptr<RoverModel> roverModel;
        boost::shared_ptr<QLearner> rlAgent;
        // Counts the time between the action and its result
        common::Timer actionTimer;
        common::Time actionInterval;
        math::Vector3 setPoint;
        unsigned numStates;
    };
    GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}
