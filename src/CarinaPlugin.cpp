#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>

using namespace std;
using namespace gazebo;


CarinaPlugin::CarinaPlugin() : steeringAngle(0), vehicleVelocity(0) {}


CarinaPlugin::~CarinaPlugin() {}


void CarinaPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "CarinaPlugin");
    const unsigned bufferSize = 1;
    // Ros topics will be used to exchange data.
    ros::NodeHandle rosNode;
    actionSubscriber = rosNode.subscribe("/rl/action", bufferSize, &CarinaPlugin::actionCallback, this);
    steeringSubscriber = rosNode.subscribe("/steering_angle", bufferSize, &CarinaPlugin::steeringCallback, this);
    rewardPublisher = rosNode.advertise<std_msgs::Float32>("/rl/reward", bufferSize);
    statePublisher = rosNode.advertise<geometry_msgs::Point32>("/rl/state", bufferSize);

    async_ros_spin.reset(new ros::AsyncSpinner(0));
    async_ros_spin->start();

    carinaModel = model;
    sdfFile = sdf;

    loadParameters();

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CarinaPlugin::onUpdate, this, _1));
}


void CarinaPlugin::onUpdate( const common::UpdateInfo &info )
{
    carinaModel->SetLinearVel( math::Vector3(vehicleVelocity, 0, 0) );
    rewardPublisher.publish( getReward() );
    statePublisher.publish( getState() );
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


void CarinaPlugin::actionCallback(const std_msgs::Int32::ConstPtr &actionMsg)
{
    // The vehicle points to the X axis
    // It can go foward and backward (action can be negative)
    int action;
    switch( actionMsg->data ){
    case(0):
        action = -1;
        break;
    case(1):
        action = 0;
        break;
    case(2):
        action = 1;
        break;
    }
    float simulationFactor = 0.1;
    vehicleVelocity = simulationFactor * action;
}


void CarinaPlugin::steeringCallback(const std_msgs::Float32::ConstPtr& steeringMsg)
{
    // The steering angle goes from -x to +x where x is the max angle
    // between the wheel and the car x axis (front of the car)
    steeringAngle = steeringMsg->data;
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


// Throttle is the device that controls the amount of gas that goes to the engine.
void CarinaPlugin::applyThrottle(const int& action)
{
    // Define the relation between throttle and impulse force
    // This depends on: car horsepower, car weight, fuel type...
    int simulationFactor = 50;
    float carPower = 1.0;
    float impulseForce = carPower * simulationFactor * action;
//    inpulseForce push the vehicle forward.
//    chassisLink->AddLinkForce( math::Vector3(impulseForce, 0, 0) );
}


const std_msgs::Float32 CarinaPlugin::getReward() const
{
    math::Vector3 setPoint(1.5, 0, 0.1); // 1.5m from world frame origin
    math::Vector3 absPosition = carinaModel->GetWorldPose().pos;
    float distance = absPosition.Distance( setPoint );
//    reward.data = abs(setPoint - absPosition.x) > 0.01 ? -1 : 0;
//    reward.data = - distance / ( distance + 4);
    std_msgs::Float32 reward;
    reward.data = - distance;

    return reward;
}


const geometry_msgs::Point32 CarinaPlugin::getState() const
{
    const float gridSize = 0.1;
    math::Vector3 absPosition = carinaModel->GetWorldPose().pos;
    geometry_msgs::Point32 state;
    state.x = static_cast<int>(absPosition.x / gridSize);
    state.y = static_cast<int>(absPosition.y / gridSize);
    state.z = static_cast<int>(absPosition.z / gridSize);

    return state;
}
