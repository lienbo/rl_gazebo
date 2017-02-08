#include <boost/make_shared.hpp>
#include "RoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RoverPlugin::RoverPlugin() : numSteps(0), maxSteps(5000), train(true)
{
    destinationPos.push_back( math::Vector3(0, 0, 0.1) );

    // Forward and backward
    initialPos.push_back( math::Pose(-3, 0, .12, 0, 0, 0) );
//    initialPos.push_back( math::Pose(2, 0, .12, 0, 0, 0) );

    // Turn left and right
    initialPos.push_back( math::Pose(-3, -2, .12, 0, 0, 0) );
    initialPos.push_back( math::Pose(-3, 2, .12, 0, 0, 0) );
//    initialPos.push_back( math::Pose(3, -2, .12, 0, 0, 0) );
//    initialPos.push_back( math::Pose(3, 2, .12, 0, 0, 0) );
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
        action = rlAgent->updateAction( state_index, action );
    }else{
        action = rlAgent->chooseAction( state_index, false );
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
        gzmsg << "Step = " << numSteps << endl;

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
            action = rlAgent->updateAction( state_index, action );

            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;
        }

        roverModel->endStep();
        ++numSteps;

        // Terminate simulation after maxSteps
        if( numSteps == maxSteps ){
            rlAgent->savePolicy();

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
        const unsigned action = rlAgent->chooseAction( state_index, false );
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
