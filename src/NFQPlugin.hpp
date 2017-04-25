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

#include "CaffeRL.hpp"
#include "RoverModel.hpp"


namespace gazebo{
    class NFQPlugin : public ModelPlugin{
        public:
        NFQPlugin();
        ~NFQPlugin();

        private:
        void Load( physics::ModelPtr model, sdf::ElementPtr sdfPtr );
        void loadParameters( const sdf::ElementPtr &sdfPtr );
        void onUpdate( const common::UpdateInfo &info );
        void firstAction();
        void trainAlgorithm();
        void testAlgorithm();

        std::vector<math::Pose> initialPos;
        std::vector<math::Vector3> destinationPos;

        transport::PublisherPtr serverControlPub;
        event::ConnectionPtr updateConnection;

        boost::shared_ptr<RoverModel> roverModel;
        boost::shared_ptr<CaffeRL> caffeNet;

        // Counts the time between the action and its result
        common::Time timeMark;
        physics::WorldPtr worldPtr;

        unsigned previousAction;
        std::vector<float> previousState;
        std::vector<Transition> transitionsContainer;

        const std::string outputDir;
        unsigned memoryReplaySize, batchSize;
        unsigned maxSteps, numSteps;
        bool train;
    };
    GZ_REGISTER_MODEL_PLUGIN(NFQPlugin)
}
