#include <boost/filesystem.hpp>
#include "RoverModel.hpp"
#include <sstream>

using namespace std;
using namespace gazebo;


RoverModel::RoverModel( physics::ModelPtr model, sdf::ElementPtr sdf ) :
        steeringState(0), velocityState(0), terminalStateCounter(0),
        terminalDistance(0.1),  distanceCounter(0),
        uniformDist(0, Action::NUM_ACTIONS - 1),
        outputDir("./gazebo/output/images/")
{
    modelPtr = model;
    sdfFile = sdf;

    setPoint.Set(0, 0, 0);
    lastDistance = getDestinationDistance();

    loadParameters();
    initializeContacts();
    initializeCamera();
    selectSimulationSpeed();
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


const unsigned RoverModel::bestAction() const
{
    const float distance = getDestinationDistance();
    const float angle = getAngletoDestination();

    // Default value => go forward
    unsigned action = Action::INCREASE_SPEED;
    if( velocityState > 0)
        action = Action::DO_NOTHING;

    // Center steering wheel
    const float angle_limit = 15;
    if( abs(angle) <= angle_limit ){
        if( steeringState > 0 ){
            action = Action::TURN_RIGHT;
        }
        if( steeringState < 0 ){
            action = Action::TURN_LEFT;
        }
    }

    // Turn steering wheel towards destination
    if(( angle > angle_limit )&&( steeringState < 5 )){
        action = Action::TURN_LEFT;
    }
    if(( angle < -angle_limit )&&( steeringState > -5 )){
        action = Action::TURN_RIGHT;
    }

    // Destination behind the vehicle
    if( abs(angle) > 60 ){
        action = Action::DECREASE_SPEED;
        if ( velocityState < 0 ){
            action = Action::DO_NOTHING;
        }
        if(( angle > 0 )&&( steeringState > -5 )){
            action = Action::TURN_RIGHT;
        }
        if(( angle < 0 )&&( steeringState < 5 )){
            action = Action::TURN_LEFT;
        }
    }

    // Rover in terminal state
    if( abs(distance) <= terminalDistance ){
        if( velocityState == 0)
            action = Action::DO_NOTHING;
        if( velocityState < 0)
            action = Action::INCREASE_SPEED;
        if( velocityState > 0)
            action = Action::DECREASE_SPEED;
    }

    return action;
}


const unsigned RoverModel::eGreedy( unsigned &action, const float &probability )
{
    bernoulli_distribution bernoulli( probability );
    // Bernoulli distribution to change action
    if( bernoulli(generator) ){
        // Equal (uniform) probability to choose an action
        action = uniformDist(generator);
    }
    return action;
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
    case( Action::INCREASE_SPEED ):
        if( velocityState < speed_limit )
            velocityState += 1;
        break;
    case( Action::DECREASE_SPEED ):
        if( velocityState > - speed_limit )
            velocityState += - 1;
        break;
    case( Action::TURN_LEFT ):
        if( steeringState < angle_limit )
            steeringState += 1;
        break;
    case( Action::TURN_RIGHT ):
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
    const float vehicle_velocity = simulationFactor * velocityState;

    rearRightJoint->SetVelocity( 0, vehicle_velocity );
    rearLeftJoint->SetVelocity( 0, vehicle_velocity );
}


void RoverModel::steeringWheelController() const
{
    // Steering wheel proportional controller.
    // steering_angle is the setpoint in RADIANS
    // max steering_angle (in degrees) = angle_limit * angle_rate
    const float angle_rate = 3;
    const float one_degree = 0.01745;
    const float steering_angle = steeringState * angle_rate * one_degree;
    const unsigned int rotation_axis = 0;
    const math::Angle current_angle = frontLeftJoint->GetAngle( rotation_axis );
    float angular_velocity = 0.2;

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


const unsigned RoverModel::getNumActions() const
{
    return Action::NUM_ACTIONS;
}


const float RoverModel::getDestinationDistance() const
{
    math::Vector3 abs_position = modelPtr->GetWorldPose().pos;
    const float distance = abs_position.Distance( setPoint );

    return distance;
}


const float RoverModel::getAngletoDestination() const
{
    math::Pose global_pose = modelPtr->GetWorldPose();
    math::Vector3 abs_position = global_pose.pos;
    math::Quaternion orientation = global_pose.rot;

    math::Vector3 relative_position = orientation.RotateVectorReverse( abs_position );
    math::Vector3 destination_vector = setPoint - relative_position;

    destination_vector = destination_vector.Normalize();

    // Angle between to vectors
    // Angle is positive counter clock wise
    // Positive -> destination on left side
    // Negative -> destination on the right
    // -pi <= angle < +pi
    float angle_radians = 0;
    if( destination_vector.x >= 0 ){
        angle_radians = asin( destination_vector.y );
    } else {
        if( destination_vector.y >= 0 ){
            angle_radians = 3.1416 - asin( destination_vector.y );
        } else {
            angle_radians = -3.1416 - asin( destination_vector.y );
        }
    }

    const float one_radian = 57.2958;
    const float angle_degrees = one_radian * angle_radians;

    return angle_degrees;
}


const float RoverModel::getReward() const
{
//    const float distance = getDestinationDistance();
//    reward = abs(setPoint - abs_position.x) > 0.01 ? -1 : 0;
//    reward = - distance / ( distance + 4);

    const float current_distance = floor( getDestinationDistance() * 10 ) / 10;
    float reward = - current_distance;

//    // The distance must have 2 decimal places due to precision fluctuations.
//    const float current_distance = floor( getDestinationDistance() * 100 ) / 100;

//    // reward = 0 if model is approaching destination, -1 if going away from it
//    const float floor_last_distance = floor(lastDistance * 100) / 100;
//    float reward = (  current_distance < floor_last_distance ) ? 0 : -1;

    // Terminal state reward
    if( current_distance < terminalDistance ){
        reward = 10;
    }

//    gzmsg << "lastDistance = " <<  round_last_distance  << endl;
    gzmsg << "distance = " << current_distance << endl;
    gzmsg << "reward = " << reward << endl;

    return reward;
}


const bool RoverModel::isTerminalState()
{
    const float distance = getDestinationDistance();

    if( distance < terminalDistance ){
        ++terminalStateCounter;
    }else{
        terminalStateCounter = 0;
    }

    // isTerminalState will only return true if we have X terminal states in a row
    // This is required to the agent learn to stop when it reaches the terminal state
    bool terminal_state = false;
    if( terminalStateCounter >= 10 )
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

    bool wrong_direction = false;
    if( distanceCounter >= 12 )
        wrong_direction = true;

    return wrong_direction;
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


const math::Vector3 RoverModel::getEulerAnglesState() const
{
    const float grid_size = 0.1;
    math::Quaternion rotation = modelPtr->GetWorldPose().rot;
    math::Vector3 euler_angles;
    euler_angles.x = round( rotation.GetRoll() / grid_size );
    euler_angles.y = round( rotation.GetPitch() / grid_size );
    euler_angles.z = round( rotation.GetYaw() / grid_size );

    return euler_angles;
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


vector<float> RoverModel::getState() const
{
    vector<float> observed_state;
    vector<string> state_names;

	// distance < 0.1 => state = 0
    // 0.1 < distance < 0.3  =>> state = 1
    // 0.3 < distance < 0.5  =>> state = 2
    state_names.push_back("distance");
    const float distance = getDestinationDistance();
    observed_state.push_back( floor((distance + 0.1)/0.2) );

    state_names.push_back("angle");
    const float angle = getAngletoDestination();
    observed_state.push_back( round(angle/18) );

//    state_names.push_back("distance");
//    const math::Vector3 distance = getDistanceState();
//    observed_state.push_back( distance.x );
//    observed_state.push_back( distance.y );
//    observed_state.push_back( distance.z );

//    state_names.push_back("orientation");
//    const math::Vector3 orientation = getEulerAnglesState();
//    In this dataset the robot wont change roll and pitch
//    observed_state.push_back( orientation.x );
//    observed_state.push_back( orientation.y );
//    observed_state.push_back( orientation.z );

    state_names.push_back("vel");
    const int velocity = getVelocityState();
    observed_state.push_back( velocity );

    state_names.push_back("steering");
    const int steering = getSteeringState();
    observed_state.push_back( steering );

    printState( observed_state, state_names );

    return observed_state;
}


void RoverModel::printState( const vector<float> &observed_state, const vector<string> &state_names ) const
{
    vector<float>::const_iterator it;
    gzmsg << "State:" << endl;
    for( size_t i = 0; i != observed_state.size(); ++i )
        gzmsg << '\t' << state_names.at(i) << " = " << observed_state.at(i) << endl;
}


void RoverModel::endStep()
{
    lastDistance = getDestinationDistance();
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


bool RoverModel::checkCollision() const
{
    bool collision = false;
    msgs::Contacts contact_msgs;
    for(ContactContainer::const_iterator it = contactPtrs.begin();
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


void RoverModel::setOriginAndDestination( const vector<math::Pose> &initial_pos,
                                          const vector<math::Vector3> &destination_pos )
{
    initialPos = initial_pos;
    destinationPos = destination_pos;
}


void RoverModel::resetModel()
{
    velocityState = 0;
    steeringState = 0;

    // Reset whole world to avoid problem with the physics engine
    modelPtr->GetWorld()->Reset();

    uniform_int_distribution<int> init_uniform_dist(0, initialPos.size() - 1);
    const unsigned new_pose = init_uniform_dist(generator);
    modelPtr->SetWorldPose( initialPos[new_pose] );

    uniform_int_distribution<int> dest_uniform_dist(0, destinationPos.size() - 1);
    const unsigned new_destination = dest_uniform_dist(generator);
    setPoint = destinationPos[new_destination];

    gzmsg << "Reseting model to initial position." << endl;
    gzmsg << endl;

    lastDistance = getDestinationDistance();
    distanceCounter = 0;
}


void RoverModel::selectSimulationSpeed( const std::string speed )
{
    if( speed == "slow" ){
        actionInterval.Set(1, 0);
        simulationFactor = 1;
    }

    if ( speed == "normal" ){
        actionInterval.Set(0, 500000000);
        simulationFactor = 2;
    }

    if ( speed == "fast" ){
        actionInterval.Set(0, 250000000);
        simulationFactor = 3;
    }
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


common::Time RoverModel::getActionInterval() const
{
    return actionInterval;
}
