// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo (USP)
// Laboratorio de robotica movel (LRM)
// Febuary 2017

// Neural Fitted Q-learning (NFQ) like implementation

// Martin Riedmiller. Neural fitted q iteration–first experiences with a data
// efficient neural re-inforcement learning method.
// InMachine Learning: ECML 2005


#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "CaffeInference.hpp"
#include "RoverModel.hpp"
#include "QLearner.hpp"


namespace gazebo{
    class NFQPlugin : public ModelPlugin{
        public:
        NFQPlugin();
        ~NFQPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdfPtr );
        void loadParameters( const physics::ModelPtr &model, const sdf::ElementPtr &sdfPtr );
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
        boost::shared_ptr<CaffeInference> caffeNet;

        // Counts the time between the action and its result
        common::Time actionInterval, timeMark;
        physics::WorldPtr worldPtr;

        unsigned maxSteps, numSteps;
        bool train;
    };
    GZ_REGISTER_MODEL_PLUGIN(NFQPlugin)
}
