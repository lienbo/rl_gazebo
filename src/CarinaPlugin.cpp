#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>


using namespace std;
using namespace gazebo;

void CarinaPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    carinaModel = model;
    sdfFile = sdf;

    loadParameters();

    // Angles are in radians. Positive is counterclockwise
    steeringAngle = 0;

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CarinaPlugin::onUpdate, this, _1));

    // The node will be used to receive data from external processes.
    node = transport::NodePtr(new transport::Node());
    node->Init();
    // Listen to Gazebo world_stats topic
    throttleSubscriber = node->Subscribe("~/" + carinaModel->GetName() +
        "/trottle_percentage", &CarinaPlugin::throttleCallback, this);

    steeringSubscriber = node->Subscribe("~/" + carinaModel->GetName() +
        "/steering_angle", &CarinaPlugin::steeringCallback, this);
}


void CarinaPlugin::onUpdate( const common::UpdateInfo &info )
{
    steeringWheelController();
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


void CarinaPlugin::steeringCallback( SteeringPtr &steeringMsg )
{
    // The steering angle goes from -x to +x where x is the max angle
    // between the wheel and the car x axis (front of the car)
    steeringAngle = steeringMsg->steering_angle();
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


void CarinaPlugin::checkParameterName( const string &parameterName )
{
    // Quit application if parameterName is not defined in sdf file
    if( !sdfFile->HasElement( parameterName.c_str() ) ){
        string msg = "CarinaPlugin: " + parameterName + " parameter not defined in model sdf file.";
        gzthrow( msg );
    }
}


void CarinaPlugin::steeringWheelController()
{
    // Steering wheel proportional controller.
    // steeringAngle is the setpoint
    const unsigned int rotationAxis = 0;
    const math::Angle currentAngle = frontLeftJoint->GetAngle( rotationAxis );
    double velocity;

    // Apply a velocity to Z axis until the wheel reaches steeringAngle
    ( currentAngle >= 0.01 + steeringAngle ) ? velocity = -5.0 : velocity = 0.0;
    ( currentAngle <= steeringAngle - 0.01 ) ? velocity = 5.0 : velocity = 0.0;
    frontLeftJoint->SetVelocity( rotationAxis, velocity );
    frontRightJoint->SetVelocity( rotationAxis, velocity );
}
