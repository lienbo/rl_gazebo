#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>


using namespace std;
using namespace gazebo;

void CarinaPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    carinaModel = model;
    sdfFile = sdf;

    loadParameters();

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CarinaPlugin::onUpdate, this, _1));

    // The node will be used to receive data from external processes.
    node = transport::NodePtr(new transport::Node());
    node->Init();
    // Listen to Gazebo world_stats topic
    throttleSubscriber = node->Subscribe("~/" + carinaModel->GetName() +
        "/acceleration_force", &CarinaPlugin::throttleCallback, this);
}


void CarinaPlugin::onUpdate( const common::UpdateInfo &info )
{
    // Do nothing
}


// Throttle is the device that controls the amount of gas that goes to the engine.
void CarinaPlugin::throttleCallback( ThrottlePtr &throttleMsg )
{
    // Define the relation between throttle and impulse force
    // This depends on: car horsepower, car weight, fuel type...
    int simulationFactor = 100;
    float carPower = 2.0;
    float impulseForce = carPower * simulationFactor * throttleMsg->throttle_percentage();
    // inpulseForce push the vehicle forward.
    // The direction is controlled by the steering wheel.
    chassisLink->AddLinkForce( math::Vector3(impulseForce, 0, 0) );
}


void CarinaPlugin::loadParameters()
{
    checkParameterName( "chassis_name" );
    chassisLink = carinaModel->GetLink( sdfFile->Get<string>("chassis_name") );
    if( !chassisLink ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("chassis_name") +
            " not found in " + carinaModel->GetName() + " model";
        gzthrow( msg );
    }

    checkParameterName( "front_left_joint" );
    frontLeftJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_left_joint") );
    if( !frontLeftJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("front_left_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzthrow( msg );
    }

    checkParameterName( "front_right_joint" );
    frontRightJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_right_joint") );
    if( !frontRightJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("front_right_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzthrow( msg );
    }
}


void CarinaPlugin::checkParameterName( string parameterName )
{
    // Quit application if parameterName is not defined in sdf file
    if( !sdfFile->HasElement( parameterName.c_str() ) ){
        string msg = "CarinaPlugin: " + parameterName + " parameter not defined in model sdf file.";
        gzthrow( msg );
    }
}
