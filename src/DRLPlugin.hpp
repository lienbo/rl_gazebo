// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include "CaffeInference.hpp"
#include "RoverModel.hpp"


namespace gazebo{
    class DRLPlugin : public ModelPlugin{
        public:
        DRLPlugin();
        ~DRLPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
        void onUpdate( const common::UpdateInfo &info );
        void printState( const std::vector<float> &observed_state );
        std::vector<float> getState();
        void trainAlgorithm();
        void runAlgorithm();

        transport::PublisherPtr serverControlPub;
        event::ConnectionPtr updateConnection;
        boost::shared_ptr<RoverModel> roverModel;
        boost::shared_ptr<CaffeInference> caffeNet;
        // Counts the time between the action and its result
        common::Timer actionTimer;
        common::Time actionInterval;
        math::Vector3 setPoint;
        unsigned maxSteps, numSteps;
    };
    GZ_REGISTER_MODEL_PLUGIN(DRLPlugin)
}
