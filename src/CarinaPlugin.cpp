#include "CarinaPlugin.hpp"
#include <gazebo/transport/transport.hh>

using namespace std;
using namespace gazebo;


CarinaPlugin::CarinaPlugin() : oneDegree(0.01745), steeringState(0), velocityState(0) {}


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
    rewardPublisher = rosNode.advertise<std_msgs::Float32>("/rl/reward", bufferSize);
    positionStatePublisher = rosNode.advertise<geometry_msgs::Point32>("/rl/state/position", bufferSize);
    velocityStatePublisher = rosNode.advertise<std_msgs::Int32>("/rl/state/velocity/", bufferSize);
    steeringStatePublisher = rosNode.advertise<std_msgs::Int32>("rl/state/steering/",bufferSize);

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
    velocityController();
    steeringWheelController();

    rewardPublisher.publish( getReward() );
    positionStatePublisher.publish( getPositionState() );
    velocityStatePublisher.publish( getVelocityState() );
    steeringStatePublisher.publish( getSteeringState() );
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

    checkParameterName( "rear_left_joint" );
    rearLeftJoint = carinaModel->GetJoint( sdfFile->Get<string>("rear_left_joint") );
    if( !rearLeftJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("rear_left_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "rear_right_joint" );
    rearRightJoint = carinaModel->GetJoint( sdfFile->Get<string>("rear_right_joint") );
    if( !rearRightJoint ){
        string msg = "CarinaPlugin: " + sdfFile->Get<string>("rear_right_joint") +
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
    const int speed_limit = 1;
    const int angle_limit = 5;

    switch( actionMsg->data ){
    case(0):
        // Emergency brake
        velocityState = 0;
        break;
    case(1):
        if( velocityState < speed_limit )
            velocityState += 1;
        break;
    case(2):
        if( velocityState > - speed_limit )
            velocityState += - 1;
        break;

    case(3):
        if( steeringState < angle_limit )
            steeringState += 1;
        break;
    case(4):
        if( steeringState > - angle_limit )
            steeringState += - 1;
        break;
    default:
        gzmsg << "Undefined action !!! " << endl;
        break;
    }
}


void CarinaPlugin::velocityController() const
{
    const float simulation_factor = 1;
    const float vehicle_velocity = simulation_factor * velocityState;

    rearRightJoint->SetVelocity( 0, vehicle_velocity );
    rearLeftJoint->SetVelocity( 0, vehicle_velocity );
}


void CarinaPlugin::steeringWheelController()
{
    // Steering wheel proportional controller.
    // steering_angle is the setpoint in RADIANS
    // max steering_angle = angle_limit * angle_rate * oneDegree
    const float angle_rate = 4;
    const float steering_angle = velocityState * angle_rate * oneDegree;
    const unsigned int rotation_axis = 0;
    const math::Angle current_angle = frontLeftJoint->GetAngle( rotation_axis );
    float angular_velocity = 0.1;

    // Apply a velocity to Z axis until the wheel reaches steeringAngle
    if( current_angle >= 0.01 + steering_angle ){
        angular_velocity *= -1;
    } else if ( current_angle <= steering_angle - 0.01 ){
        angular_velocity *= 1;
    } else {
        angular_velocity = 0.0;
    }
    frontLeftJoint->SetVelocity( rotation_axis, angular_velocity );
    frontRightJoint->SetVelocity( rotation_axis, angular_velocity );
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
    math::Vector3 setPoint(3.5, 3.5, 0.1); // 1.5m from world frame origin
    math::Vector3 absPosition = carinaModel->GetWorldPose().pos;
    float distance = absPosition.Distance( setPoint );
//    reward.data = abs(setPoint - absPosition.x) > 0.01 ? -1 : 0;
//    reward.data = - distance / ( distance + 4);
    std_msgs::Float32 reward;
    reward.data = - distance;

    return reward;
}


const geometry_msgs::Point32 CarinaPlugin::getPositionState() const
{
    const float gridSize = 0.2;
    math::Vector3 absPosition = carinaModel->GetWorldPose().pos;
    geometry_msgs::Point32 state;
    state.x = round(absPosition.x / gridSize);
    state.y = round(absPosition.y / gridSize);
    state.z = round(absPosition.z / gridSize);

    return state;
}


const std_msgs::Int32 CarinaPlugin::getVelocityState() const
{
    std_msgs::Int32 velocity_state;
    velocity_state.data = velocityState;

    return velocity_state;
}


const std_msgs::Int32 CarinaPlugin::getSteeringState() const
{
    std_msgs::Int32 steering_state;
    steering_state.data = steeringState;

    return steering_state;
}
