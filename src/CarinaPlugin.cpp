#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>

using namespace std;
using namespace gazebo;


CarinaPlugin::CarinaPlugin() : steeringAngle(0) {}


CarinaPlugin::~CarinaPlugin() {}


void CarinaPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "CarinaPlugin");
    // Ros topics will be used to exchange data.
    ros::NodeHandle rosNode;
    actionSubscriber = rosNode.subscribe("/reinforcement_learning/action", 10, &CarinaPlugin::actionCallback, this);
    steeringSubscriber = rosNode.subscribe("/steering_angle", 10, &CarinaPlugin::steeringCallback, this);

    async_ros_spin.reset(new ros::AsyncSpinner(1));
    async_ros_spin->start();

    carinaModel = model;
    sdfFile = sdf;

    loadParameters();

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CarinaPlugin::onUpdate, this, _1));
}


void CarinaPlugin::actionCallback(const std_msgs::String::ConstPtr &actionMsg)
{
    applyThrottle(1);
}


void CarinaPlugin::onUpdate( const common::UpdateInfo &info )
{
    //steeringWheelController();
}


// Throttle is the device that controls the amount of gas that goes to the engine.
void CarinaPlugin::applyThrottle(const int& action)
{
    // Define the relation between throttle and impulse force
    // This depends on: car horsepower, car weight, fuel type...
    int simulationFactor = 10;
    float carPower = 1.0;
    float impulseForce = carPower * simulationFactor * action;
    // inpulseForce push the vehicle forward.
    chassisLink->AddLinkForce( math::Vector3(impulseForce, 0, 0) );
}


void CarinaPlugin::steeringCallback(const std_msgs::Float32::ConstPtr& steeringMsg)
{
    // The steering angle goes from -x to +x where x is the max angle
    // between the wheel and the car x axis (front of the car)
    steeringAngle = steeringMsg->data;
}


void CarinaPlugin::loadParameters()
{
    checkParameterName( "chassis_name" );
    chassisLink = carinaModel->GetLink( sdfFile->Get<string>("chassis_name") );
    if( !chassisLink ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("chassis_name") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_left_joint" );
    frontLeftJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_left_joint") );
    if( !frontLeftJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("front_left_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_right_joint" );
    frontRightJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_right_joint") );
    if( !frontRightJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("front_right_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
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
