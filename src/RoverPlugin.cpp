#include <boost/make_shared.hpp>
#include "RoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RoverPlugin::RoverPlugin() {}

RoverPlugin::~RoverPlugin() {}

void RoverPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    roverModel = boost::make_shared<RoverModel>(model, sdf);
    rlAgent = boost::make_shared<QLearner>();

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RoverPlugin::onUpdate, this, _1));
}


void RoverPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();

    vector<float> observed_state = getState();
    const unsigned action = rlAgent->chooseAction( observed_state );
    roverModel->applyAction(action);

    // Wait few seconds
    observed_state = getState();
    math::Vector3 set_point(3.5, 3.5, 0.1);
    rlAgent->updateQValues( roverModel->getReward(set_point), observed_state );
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
