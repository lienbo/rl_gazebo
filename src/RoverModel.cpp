#include "RoverModel.hpp"

using namespace std;
using namespace gazebo;


RoverModel::RoverModel(physics::ModelPtr model, sdf::ElementPtr sdf) :
        steeringState(0), velocityState(0)
{
    carinaModel = model;
    sdfFile = sdf;
    loadParameters();
}


RoverModel::~RoverModel() {}


void RoverModel::loadParameters()
{
    checkParameterName( "chassis_name" );
    chassisLink = carinaModel->GetLink( sdfFile->Get<string>("chassis_name") );
    if( !chassisLink ){
        string msg = "RoverModel: " + sdfFile->Get<string>("chassis_name") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_left_joint" );
    frontLeftJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_left_joint") );
    if( !frontLeftJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("front_left_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_right_joint" );
    frontRightJoint = carinaModel->GetJoint( sdfFile->Get<string>("front_right_joint") );
    if( !frontRightJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("front_right_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "rear_left_joint" );
    rearLeftJoint = carinaModel->GetJoint( sdfFile->Get<string>("rear_left_joint") );
    if( !rearLeftJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("rear_left_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "rear_right_joint" );
    rearRightJoint = carinaModel->GetJoint( sdfFile->Get<string>("rear_right_joint") );
    if( !rearRightJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("rear_right_joint") +
            " not found in " + carinaModel->GetName() + " model";
        gzmsg << msg << endl;
    }
}


void RoverModel::checkParameterName( const string &parameter_name )
{
    // Quit application if parameterName is not defined in sdf file
    if( !sdfFile->HasElement( parameter_name.c_str() ) ){
        string msg = "RoverModel: " + parameter_name + " parameter not defined in model sdf file.";
        gzthrow( msg );
    }
}


void RoverModel::applyAction(const int &action)
{
    // The vehicle points to the X axis
    // It can go foward and backward (action can be negative)
    const int speed_limit = 1;
    const int angle_limit = 5;

    switch( action ){
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


void RoverModel::velocityController() const
{
    const float simulation_factor = 1;
    const float vehicle_velocity = simulation_factor * velocityState;

    rearRightJoint->SetVelocity( 0, vehicle_velocity );
    rearLeftJoint->SetVelocity( 0, vehicle_velocity );
}


void RoverModel::steeringWheelController()
{
    // Steering wheel proportional controller.
    // steering_angle is the setpoint in RADIANS
    // max steering_angle (in degrees) = angle_limit * angle_rate
    const float angle_rate = 4;
    const float one_degree = 0.01745;
    const float steering_angle = steeringState * angle_rate * one_degree;
    const unsigned int rotation_axis = 0;
    const math::Angle current_angle = frontLeftJoint->GetAngle( rotation_axis );
    float angular_velocity = 0.15;

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


const float RoverModel::getReward( math::Vector3 set_point ) const
{
    math::Vector3 abs_position = carinaModel->GetWorldPose().pos;
    float distance = abs_position.Distance( set_point );
//    reward = abs(set_point - abs_position.x) > 0.01 ? -1 : 0;
//    reward = - distance / ( distance + 4);
    const float reward = - distance;

    return reward;
}


const math::Vector3 RoverModel::getPositionState() const
{
    const float grid_size = 0.2;
    math::Vector3 abs_position = carinaModel->GetWorldPose().pos;
    math::Vector3 position_state;
    position_state.x = round( abs_position.x / grid_size );
    position_state.y = round( abs_position.y / grid_size );
    position_state.z = round( abs_position.z / (grid_size + 0.1));

    return position_state;
}


const math::Quaternion RoverModel::getOrientationState() const
{
    const float grid_size = 0.1;
    math::Quaternion rotation = carinaModel->GetWorldPose().rot;
    math::Quaternion orientation_state;
    orientation_state.w = round( rotation.w / grid_size );
    orientation_state.x = round( (rotation.x + 0.0001 )/ grid_size );
    orientation_state.y = round( (rotation.y + 0.0001 )/ grid_size );
    orientation_state.z = round( (rotation.z)/ grid_size );

    return orientation_state;
}


const int RoverModel::getVelocityState() const
{
    const int velocity_state = velocityState;
    return velocity_state;
}


const int RoverModel::getSteeringState() const
{
    const int steering_state = steeringState;
    return steering_state;
}
