#include <boost/make_shared.hpp>
#include "RoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RoverPlugin::RoverPlugin() {
    actionTimer.Reset();
    actionInterval.Set(1,0);
    setPoint.Set(3.5, 3.5, 0.1);
}

RoverPlugin::~RoverPlugin() {}

void RoverPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    const unsigned num_actions = 5;
    rlAgent = boost::make_shared<QLearner>( num_actions );
    roverModel = boost::make_shared<RoverModel>(model, sdf);

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RoverPlugin::onUpdate, this, _1));

    actionTimer.Start();

    // Apply first action
    vector<float> observed_state = getState();
    printState( observed_state );
    const unsigned action = rlAgent->chooseAction( observed_state );
    roverModel->applyAction(action);
    gzmsg << "Applying action = " << action << endl;
}


void RoverPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();

    common::Time elapsedTime = actionTimer.GetElapsed();
    if( elapsedTime >= actionInterval ){

        vector<float> observed_state = getState();
        const float reward = roverModel->getReward(setPoint);
        rlAgent->updateQValues( reward, observed_state );

        const unsigned action = rlAgent->chooseAction( observed_state );
        roverModel->applyAction( action );

        printState( observed_state );
        gzmsg << "Applying action = " << action << endl;

        actionTimer.Reset();
        actionTimer.Start();
    }
}


vector<float> RoverPlugin::getState()
{
    vector<float> observed_state;
    math::Vector3 position = roverModel->getPositionState();
    observed_state.push_back( position.x );
    observed_state.push_back( position.y );
    observed_state.push_back( position.z );

    math::Quaternion orientation = roverModel->getOrientationState();
    observed_state.push_back( orientation.w );
    observed_state.push_back( orientation.x );
    observed_state.push_back( orientation.y );
    observed_state.push_back( orientation.z );

    const int velocity = roverModel->getVelocityState();
    observed_state.push_back( velocity );

    const int steering = roverModel->getSteeringState();
    observed_state.push_back( steering );

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
