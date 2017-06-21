#include <boost/make_shared.hpp>
#include "RoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RoverPlugin::RoverPlugin() :
    numSteps(0), maxSteps(5000),
    numTrials(0), maxTrials(2),
    nearState(0),
    train(true)
{
    destinationPos.push_back( math::Vector3(0, 0, 0) );

    // Forward
//    initialPos.push_back( math::Pose(-5, -4, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-4, -3, 0, 0, 0, 0) );

//    initialPos.push_back( math::Pose(-3, -2, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-2, -1, 0, 0, 0, 0) );
//     initialPos.push_back( math::Pose(-3, 0, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-2, 1, 0, 0, 0, 0) );
    initialPos.push_back( math::Pose(-3, 2, 0, 0, 0, 0) );

//    initialPos.push_back( math::Pose(-4, 3, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-5, 4, 0, 0, 0, 0) );

    // Backward
//    initialPos.push_back( math::Pose(3, -2, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(2, -1, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(3, 0, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(2, 1, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(3, 2, 0, 0, 0, 0) );
}


RoverPlugin::~RoverPlugin() {}


void RoverPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdfPtr )
{
    transport::NodePtr node( new transport::Node() );
    node->Init();
    serverControlPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");

    loadParameters( sdfPtr );

    roverModel = boost::make_shared<RoverModel>( model, sdfPtr );
    roverModel->setOriginAndDestination( initialPos, destinationPos );

    rlAgent = boost::make_shared<QLearner>( roverModel->getNumActions() );
    rlAgent->loadPolicy();

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RoverPlugin::onUpdate, this, _1));

    // World simulation time will be used to synchronize the actions
    worldPtr = model->GetWorld();
    timeMark = worldPtr->GetSimTime();

    // Apply first action
    roverModel->resetModel();
    firstAction();
}


void RoverPlugin::loadParameters( const sdf::ElementPtr &sdfPtr )
{
    if( sdfPtr->HasElement( "mode" ) ){
        string execution_mode = sdfPtr->Get<string>("mode");
        if( execution_mode == "train" ){
            train = true;
        }
        if( execution_mode == "test" ){
            train = false;
        }
    }

    if( sdfPtr->HasElement( "max_steps" ) )
        maxSteps = sdfPtr->Get<unsigned>("max_steps");

    if( sdfPtr->HasElement( "max_trials" ) )
        maxTrials = sdfPtr->Get<unsigned>("max_trials");
}


void RoverPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();
    train ? trainAlgorithm() : testAlgorithm();
}


// This function doesn't call updateQValues because the initial position doesn't
// have a last state to update.
void RoverPlugin::firstAction() const
{
    vector<float> observed_state = roverModel->getState();
    const unsigned state_index = rlAgent->fetchState( observed_state );

    unsigned action;
    if(train){
        action = roverModel->bestAction();
        const float initial_epsilon = 0.7; // Final epsilon is zero
        const float epsilon = initial_epsilon*(1.0 - ( round(10. * numSteps/maxSteps) /10. ));
        action = roverModel->eGreedy( action, epsilon );
        rlAgent->updateAction( state_index, action );
    }else{
        action = rlAgent->selectAction( state_index );
    }

    roverModel->applyAction( action );
    gzmsg << "Applying action = " << action << endl;
}


void RoverPlugin::trainAlgorithm()
{
    bool collision = roverModel->checkCollision();
    if( collision ){
        gzmsg << "Collision detected !!!" << endl;
        const float bad_reward = -100;
        rlAgent->updateQValues( bad_reward );
        // Reset gazebo model to initial position
        roverModel->resetModel();
        firstAction();
    }

    common::Time elapsed_time = worldPtr->GetSimTime() - timeMark;
    if( elapsed_time >= roverModel->getActionInterval() ){
        gzmsg << endl;
        gzmsg << "Trial-step = " << numTrials << "-" << numSteps << endl;

        // Terminal state
        if( roverModel->isTerminalState() ){
            gzmsg << "Model reached terminal state !!!" << endl;
            roverModel->resetModel();
            firstAction();

        }else if( roverModel->isDistancing() ){
            // Reset model if it is going in the opposite direction
            // This helps reduce the number of states decreasing the learning time
            gzmsg << "Wrong direction !" << endl;
            roverModel->resetModel();
            firstAction();

        }else{
            const float reward = roverModel->getReward();
            vector<float> observed_state = roverModel->getState();
            const unsigned state_index = rlAgent->fetchState( observed_state );
            rlAgent->updateQValues( reward, state_index );

            unsigned action = roverModel->bestAction();
            const float initial_epsilon = 0.7; // Final epsilon is zero
            const float epsilon = initial_epsilon*(1.0 - ( round(10. * numSteps/maxSteps) /10. ));
            gzmsg << "Epsilon = " << epsilon << endl;
            action = roverModel->eGreedy( action, epsilon );
            rlAgent->updateAction( state_index, action );

            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;
        }

        roverModel->endStep();
        ++numSteps;

        if( numSteps == maxSteps ){
            numSteps = 0;
            ++numTrials;
        }

        // Terminate simulation after maxTrials
        if( numTrials == maxTrials ){
            gzmsg << endl;
            gzmsg << "Simulation reached max number of trials." << endl;

            gzmsg << "Saving policy..." << endl;
            rlAgent->savePolicy( true, false );

            gzmsg << "Terminating simulation..." << endl;
            msgs::ServerControl server_msg;
            server_msg.set_stop(true);
            serverControlPub->Publish(server_msg);
        }

        timeMark = worldPtr->GetSimTime();
    }
}

void RoverPlugin::testAlgorithm()
{
    common::Time elapsed_time = worldPtr->GetSimTime() - timeMark;
    if( elapsed_time >= roverModel->getActionInterval() ){
        if( roverModel->isTerminalState() ){
            gzmsg << "Model reached terminal state !!!" << endl;
            roverModel->resetModel();
        }

        vector<float> observed_state = roverModel->getState();
        const unsigned state_index = rlAgent->fetchState( observed_state );
        const unsigned action = rlAgent->selectAction( state_index );
        roverModel->applyAction( action );
        gzmsg << "Applying action = " << action << endl;
        gzmsg << endl;

        ++numSteps;
        // Terminate simulation after maxStep
        if( numSteps == maxSteps){
            gzmsg << endl;
            gzmsg << "Simulation reached max number of steps." << endl;
            gzmsg << "Terminating simulation..." << endl;
            msgs::ServerControl server_msg;
            server_msg.set_stop(true);
            serverControlPub->Publish(server_msg);
        }

        timeMark = worldPtr->GetSimTime();
    }
}
