// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "RoverModel.hpp"
#include "QLearner.hpp"


namespace gazebo{
    class RoverPlugin : public ModelPlugin{
        public:
        RoverPlugin();
        ~RoverPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdfPtr );
        void loadParameters( const sdf::ElementPtr &sdfPtr );
        void onUpdate( const common::UpdateInfo &info );
        void firstAction() const;
        void trainAlgorithm();
        void testAlgorithm();

        std::vector<math::Pose> initialPos;
        std::vector<math::Vector3> destinationPos;

        transport::PublisherPtr serverControlPub;
        event::ConnectionPtr updateConnection;
        boost::shared_ptr<RoverModel> roverModel;
        boost::shared_ptr<QLearner> rlAgent;
        // Counts the time between the action and its result
        common::Time timeMark;
        physics::WorldPtr worldPtr;

        unsigned maxSteps, numSteps;
        unsigned maxTrials, numTrials;
        unsigned nearState;
        bool train;
    };
    GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}
