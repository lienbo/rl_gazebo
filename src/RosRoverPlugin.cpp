#include "RosRoverPlugin.hpp"

using namespace std;
using namespace gazebo;


RosRoverPlugin::RosRoverPlugin() {}


RosRoverPlugin::~RosRoverPlugin() {}


void RosRoverPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "RosRoverPlugin");

    // Ros topics will be used to exchange data.
    ros::NodeHandle rosNode;
    const unsigned buffer_size = 1;
    actionSubscriber = rosNode.subscribe("/rl/action", buffer_size, &RosRoverPlugin::actionCallback, this);

    rewardPublisher = rosNode.advertise<std_msgs::Float32>("/rl/reward", buffer_size);
    positionStatePublisher = rosNode.advertise<geometry_msgs::Point32>("/rl/state/position", buffer_size);
    orientationStatePublisher = rosNode.advertise<geometry_msgs::Quaternion>("/rl/state/orientation", buffer_size);
    velocityStatePublisher = rosNode.advertise<std_msgs::Int32>("/rl/state/velocity/", buffer_size);
    steeringStatePublisher = rosNode.advertise<std_msgs::Int32>("/rl/state/steering/", buffer_size);

    async_ros_spin.reset(new ros::AsyncSpinner(0));
    async_ros_spin->start();

    roverModel = boost::make_shared<RoverModel>(model, sdf);

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RosRoverPlugin::onUpdate, this, _1));
}


void RosRoverPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();

    rewardPublisher.publish( getReward() );

    positionStatePublisher.publish( getPositionState() );
    orientationStatePublisher.publish( getOrientationState() );
    velocityStatePublisher.publish( getVelocityState() );
    steeringStatePublisher.publish( getSteeringState() );
}


void RosRoverPlugin::actionCallback(const std_msgs::Int32::ConstPtr &actionMsg)
{
    const int action = actionMsg->data;
    roverModel->applyAction( action );
}


const std_msgs::Float32 RosRoverPlugin::getReward() const
{
    std_msgs::Float32 reward;
    math::Vector3 set_point(3.5, 3.5, 0.1); // 1.5m from world frame origin
    reward.data = roverModel->getReward( set_point );

    return reward;
}


const geometry_msgs::Point32 RosRoverPlugin::getPositionState() const
{
    math::Vector3 position = roverModel->getPositionState(); 
    geometry_msgs::Point32 position_state;
    position_state.x = position.x;
    position_state.y = position.y;
    position_state.z = position.z;

    return position_state;
}


const geometry_msgs::Quaternion RosRoverPlugin::getOrientationState() const
{
    math::Quaternion rotation = roverModel->getOrientationState(); 
    geometry_msgs::Quaternion orientation_state;
    orientation_state.w = rotation.w;
    orientation_state.x = rotation.x;
    orientation_state.y = rotation.y;
    orientation_state.z = rotation.z;

    return orientation_state;
}


const std_msgs::Int32 RosRoverPlugin::getVelocityState() const
{
    std_msgs::Int32 velocity_state;
    velocity_state.data = roverModel->getVelocityState();

    return velocity_state;
}


const std_msgs::Int32 RosRoverPlugin::getSteeringState() const
{
    std_msgs::Int32 steering_state;
    steering_state.data = roverModel->getSteeringState();

    return steering_state;
}
