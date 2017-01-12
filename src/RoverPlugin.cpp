#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include "RoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RoverPlugin::RoverPlugin() : numStates(0)
{
    actionTimer.Reset();
    actionInterval.Set(1,0);
    setPoint.Set(5, 0, 0.1);
    // Create images output directory
    boost::filesystem::path dir( "./images/" );
    boost::filesystem::create_directory(dir);
}

RoverPlugin::~RoverPlugin()
{
    rlAgent->savePolicy();
}

void RoverPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    const unsigned num_actions = 6;
    rlAgent = boost::make_shared<QLearner>( num_actions );
    roverModel = boost::make_shared<RoverModel>(model, sdf);

    rlAgent->loadPolicy();

    cameraPtr = dynamic_pointer_cast<sensors::CameraSensor>(sensors::get_sensor("camera_sensor"));
    cameraPtr->SetActive(true);

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RoverPlugin::onUpdate, this, _1));

    actionTimer.Start();

    // Apply first action
    vector<float> observed_state = getState();
    if( rlAgent->isNewState( observed_state ) ){
        string image_name = "./images/" + to_string(numStates) + ".png";
        cameraPtr->SaveFrame( image_name );
        ++numStates;
    }
    const unsigned action = rlAgent->chooseAction( observed_state );
    roverModel->applyAction(action);
    gzmsg << "Applying action = " << action << endl;
}


void RoverPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();

    bool collision = roverModel->checkCollision();
    if( collision ){
        gzmsg << "Collision detected !!!" << endl;
        const vector<float> observed_state = getState();
        const float bad_reward = -1000;
        rlAgent->updateQValues( bad_reward, observed_state );
        // Reset gazebo model to initial position
        gzmsg << "Reseting model to initial position." << endl;
        roverModel->resetModel();
    }

    common::Time elapsedTime = actionTimer.GetElapsed();
    if( elapsedTime >= actionInterval ){
        const float reward = roverModel->getReward(setPoint);
        vector<float> observed_state = getState();
        if( rlAgent->isNewState( observed_state) ){
            string image_name = "./images/" + to_string(numStates) + ".png";
            cameraPtr->SaveFrame( image_name );
            ++numStates;
        }
        rlAgent->updateQValues( reward, observed_state );

        // Terminal state
        if( reward > -0.2 )
            roverModel->resetModel();

        const unsigned action = rlAgent->chooseAction( observed_state );
        roverModel->applyAction( action );
        gzmsg << "Applying action = " << action << endl;

        actionTimer.Reset();
        actionTimer.Start();
    }
}


vector<float> RoverPlugin::getState()
{
    vector<float> observed_state;

    math::Vector3 distance = roverModel->getDistanceState( setPoint );
    observed_state.push_back( distance.x );
    observed_state.push_back( distance.y );
    observed_state.push_back( distance.z );

    math::Quaternion orientation = roverModel->getOrientationState();
    observed_state.push_back( orientation.w );
    observed_state.push_back( orientation.x );
    observed_state.push_back( orientation.y );
    observed_state.push_back( orientation.z );

    const int velocity = roverModel->getVelocityState();
    observed_state.push_back( velocity );

    const int steering = roverModel->getSteeringState();
    observed_state.push_back( steering );

    printState( observed_state );

    return observed_state;
}


void RoverPlugin::printState( const vector<float> &observed_state )
{
    vector<float>::const_iterator it;
    gzmsg << endl;
    stringstream stream;
    stream << "State = ( ";
    for(it = observed_state.begin(); it != observed_state.end(); ++it)
        stream << setprecision(2) << *it << " ";
    stream << ")";

    gzmsg << stream.str() << endl;
}
