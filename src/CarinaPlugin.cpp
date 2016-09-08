#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>

using namespace gazebo;


void CarinaPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdfFile)
{
    carinaModel = model;

    // OnUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CarinaPlugin::OnUpdate, this, _1));

    // The node will be used to receive data from external processes.
    node = transport::NodePtr(new transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    throttleSubscriber = node->Subscribe("~/" + carinaModel->GetName() +
        "/acceleration_force", &CarinaPlugin::ThrottleCallback, this);
}


void CarinaPlugin::OnUpdate(const common::UpdateInfo &info)
{
    // Do nothing
}


// Throttle is the device that controls the amount of gas that goes to the engine.
void CarinaPlugin::ThrottleCallback(ThrottlePtr &throttleMsg)
{
    physics::LinkPtr chassisLink = carinaModel->GetLink("chassis");
    // Define the relation between throttle and impulse force
    // This depends on: car horsepower, car weight, fuel type...
    int simulationFactor = 100;
    float carPower = 2.0;
    float impulseForce = carPower * simulationFactor * throttleMsg->throttle_percentage();
    chassisLink->AddLinkForce( math::Vector3(impulseForce, 0, 0) );
}
