#include <boost/filesystem.hpp>
#include "RoverModel.hpp"
#include <sstream>

using namespace std;
using namespace gazebo;


RoverModel::RoverModel(physics::ModelPtr model, sdf::ElementPtr sdf, math::Vector3 destination_pos) :
        steeringState(0), velocityState(0), terminalStateCounter(0), outputDir("./gazebo/output/images/"),
        setPoint(destination_pos), distanceCounter(0)
{
    modelPtr = model;
    sdfFile = sdf;

    lastDistance = getDestinationDistance();
    loadParameters();
    initializeContacts();
    initializeCamera();
}


RoverModel::~RoverModel() {}


void RoverModel::loadParameters()
{
    checkParameterName( "chassis_name" );
    chassisLink = modelPtr->GetLink( sdfFile->Get<string>("chassis_name") );
    if( !chassisLink ){
        string msg = "RoverModel: " + sdfFile->Get<string>("chassis_name") +
            " not found in " + modelPtr->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_left_joint" );
    frontLeftJoint = modelPtr->GetJoint( sdfFile->Get<string>("front_left_joint") );
    if( !frontLeftJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("front_left_joint") +
            " not found in " + modelPtr->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "front_right_joint" );
    frontRightJoint = modelPtr->GetJoint( sdfFile->Get<string>("front_right_joint") );
    if( !frontRightJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("front_right_joint") +
            " not found in " + modelPtr->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "rear_left_joint" );
    rearLeftJoint = modelPtr->GetJoint( sdfFile->Get<string>("rear_left_joint") );
    if( !rearLeftJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("rear_left_joint") +
            " not found in " + modelPtr->GetName() + " model";
        gzmsg << msg << endl;
    }

    checkParameterName( "rear_right_joint" );
    rearRightJoint = modelPtr->GetJoint( sdfFile->Get<string>("rear_right_joint") );
    if( !rearRightJoint ){
        string msg = "RoverModel: " + sdfFile->Get<string>("rear_right_joint") +
            " not found in " + modelPtr->GetName() + " model";
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


void RoverModel::applyAction(const unsigned &action)
{
    // The vehicle points to the X axis
    // It can go foward and backward (action can be negative)
    const int speed_limit = 1;
    const int angle_limit = 5;

    switch( action ){
    case( Action::DO_NOTHING ):
        // Do nothing
        break;
    case( Action::FORWARD ):
        if( velocityState < speed_limit )
            velocityState += 1;
        break;
    case( Action::BACKWARD ):
        if( velocityState > - speed_limit )
            velocityState += - 1;
        break;
    case( Action::TURN_RIGHT ):
        if( steeringState < angle_limit )
            steeringState += 1;
        break;
    case( Action::TURN_LEFT ):
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


const float RoverModel::getDestinationDistance() const
{
    math::Vector3 abs_position = modelPtr->GetWorldPose().pos;
    const float distance = abs_position.Distance( setPoint );

    return distance;
}


const float RoverModel::getReward() const
{
    const float distance = getDestinationDistance();
//    reward = abs(setPoint - abs_position.x) > 0.01 ? -1 : 0;
//    reward = - distance / ( distance + 4);

    float reward = - distance;

    // Terminal state reward
    if( distance < 0.2 ){
        reward = 100;
    }

    return reward;
}


const bool RoverModel::isTerminalState()
{
    const float distance = getDestinationDistance();

    if( distance < 0.2 ){
        ++terminalStateCounter;
    }else{
        terminalStateCounter = 0;
    }

    // isTerminalState will only return true if we have X terminal states in a row
    // This is required to the agent learn to stop when it reaches the terminal state
    bool terminal_state = false;
    if( terminalStateCounter >= 4 )
        terminal_state = true;

    return terminal_state;
}


const bool RoverModel::isDistancing()
{
    const float distance = getDestinationDistance();
    if( distance >= lastDistance ){
        ++distanceCounter;
    }else{
        distanceCounter = 0;
    }
    lastDistance = distance;

    bool is_farther = false;
    if( distanceCounter >= 7 )
        is_farther = true;

    return is_farther;
}


const math::Vector3 RoverModel::getDistanceState() const
{
    const float grid_size = 0.2;
    math::Vector3 distance = modelPtr->GetWorldPose().pos;
    distance.x = round(( setPoint.x - distance.x ) / grid_size);
    distance.y = round(( setPoint.y - distance.y ) / grid_size);
    distance.z = round(( setPoint.z - distance.z ) / grid_size);

    return distance;
}


const math::Vector3 RoverModel::getPositionState() const
{
    const float grid_size = 0.2;
    math::Vector3 position_state = modelPtr->GetWorldPose().pos;
    position_state.x = round( position_state.x / grid_size );
    position_state.y = round( position_state.y / grid_size );
    position_state.z = round( position_state.z / (grid_size + 0.1));

    return position_state;
}


const math::Quaternion RoverModel::getOrientationState() const
{
    const float grid_size = 0.1;
    math::Quaternion rotation = modelPtr->GetWorldPose().rot;
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


void RoverModel::initializeContacts( )
{
    sensors::ContactSensorPtr contactPtr = dynamic_pointer_cast<sensors::ContactSensor>(sensors::get_sensor("contact_sensor_chassis"));
    contactPtrs.push_back( contactPtr );

    contactPtr = dynamic_pointer_cast<sensors::ContactSensor>(sensors::get_sensor("contact_sensor_rlw"));
    contactPtrs.push_back( contactPtr );

    contactPtr = dynamic_pointer_cast<sensors::ContactSensor>(sensors::get_sensor("contact_sensor_rrw"));
    contactPtrs.push_back( contactPtr );

    contactPtr = dynamic_pointer_cast<sensors::ContactSensor>(sensors::get_sensor("contact_sensor_flw"));
    contactPtrs.push_back( contactPtr );

    contactPtr = dynamic_pointer_cast<sensors::ContactSensor>(sensors::get_sensor("contact_sensor_frw"));
    contactPtrs.push_back( contactPtr );

    for(ContactContainer::iterator it = contactPtrs.begin();
           it != contactPtrs.end(); ++it){
        (*it)->SetActive(true);
    }
}


bool RoverModel::checkCollision()
{
    bool collision = false;
    msgs::Contacts contact_msgs;
    for(ContactContainer::iterator it = contactPtrs.begin();
            it != contactPtrs.end(); ++it){
        contact_msgs = (*it)->Contacts();
        for (unsigned int i = 0; i < contact_msgs.contact_size(); ++i){
            string contact_str_01 = contact_msgs.contact(i).collision1();
            string contact_str_02 = contact_msgs.contact(i).collision2();

            if(( contact_str_01.find("box_obstacle") != string::npos )||( contact_str_01.find("walls") != string::npos ))
                collision = true;

            if(( contact_str_02.find("box_obstacle") != string::npos )||( contact_str_02.find("walls") != string::npos ))
                collision = true;
        }
    }

    return collision;
}


void RoverModel::resetModel()
{
    velocityState = 0;
    steeringState = 0;

    modelPtr->Reset();
    gzmsg << "Reseting model to initial position." << endl;
    gzmsg << endl;

    lastDistance = getDestinationDistance();
    distanceCounter = 0;
}


void RoverModel::resetModel( vector<math::Pose> initial_pos, vector<math::Vector3> destination_pos )
{
    velocityState = 0;
    steeringState = 0;

    modelPtr->Reset();

    uniform_int_distribution<int> init_uniform_dist(0, initial_pos.size() - 1);
    const unsigned new_pose = init_uniform_dist(generator);
    modelPtr->SetWorldPose( initial_pos[new_pose] );

    uniform_int_distribution<int> dest_uniform_dist(0, destination_pos.size() - 1);
    const unsigned new_destination = dest_uniform_dist(generator);
    setPoint = destination_pos[new_destination];

    gzmsg << "Reseting model to initial position." << endl;
    gzmsg << endl;

    lastDistance = getDestinationDistance();
    distanceCounter = 0;
}


void RoverModel::initializeCamera()
{
    // Create images output directory
    boost::filesystem::path dir( outputDir.c_str() );
    boost::filesystem::create_directories( dir );

    cameraPtr = dynamic_pointer_cast<sensors::CameraSensor>(sensors::get_sensor("camera_sensor"));
    cameraPtr->SetActive(true);
}


void RoverModel::saveImage( const unsigned &state_index ) const
{
    ostringstream image_name;
    image_name << outputDir.c_str() << setfill('0') << setw(8) << state_index << ".png";
    cameraPtr->SaveFrame( image_name.str() );
}


const unsigned char* RoverModel::getImage() const
{
    return cameraPtr->ImageData();
}

const unsigned RoverModel::getImageHeight() const
{
    return cameraPtr->ImageHeight();
}

const unsigned RoverModel::getImageWidth() const
{
    return cameraPtr->ImageWidth();
}


const unsigned RoverModel::getNumActions() const
{
    return Action::NUM_ACTIONS;
}
