#include <gazebo/msgs/server_control.pb.h>
#include <boost/make_shared.hpp>
#include "DRLPlugin.hpp"

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace gazebo;


DRLPlugin::DRLPlugin() : numSteps(0), trainNet(true)
{
    actionTimer.Reset();
    actionInterval.Set(1,0);
    setPoint.Set(2, 0, 0.1);
}

DRLPlugin::~DRLPlugin() {}

void DRLPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    maxSteps = 1000;
    transport::NodePtr node( new transport::Node() );
    node->Init();
    serverControlPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");

    const unsigned num_actions = 6;
    rlAgent = boost::make_shared<QLearner>( num_actions );
    roverModel = boost::make_shared<RoverModel>( model, sdf );

    const string model_file = "./caffe/network/drl_gazebo.prototxt";
    const string trained_file = "./caffe/models/drl_gazebo_iter_500.caffemodel";
    caffeNet = boost::make_shared<CaffeInference>( model_file, trained_file );

    if( sdf->HasElement( "train" ) )
        trainNet = sdf->Get<bool>("train");

    if( sdf->HasElement( "max_steps" ) )
        maxSteps = sdf->Get<unsigned>("max_steps");

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DRLPlugin::onUpdate, this, _1));

    actionTimer.Start();

    // Apply first action
    vector<float> observed_state = getState();
    const unsigned state_index = rlAgent->fetchState( observed_state );
    const unsigned action = rlAgent->chooseAction( state_index, true );
    roverModel->applyAction( action );
    gzmsg << "Applying action = " << action << endl;
}


void DRLPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();
    trainNet ? trainAlgorithm(): runAlgorithm();
}


vector<float> DRLPlugin::getState()
{
    vector<float> observed_state;

    math::Vector3 distance = roverModel->getDistanceState( setPoint );
    observed_state.push_back( distance.x );
    observed_state.push_back( distance.y );
    observed_state.push_back( distance.z );

    math::Quaternion orientation = roverModel->getOrientationState();
    observed_state.push_back( orientation.w );
//    observed_state.push_back( orientation.x );
//    observed_state.push_back( orientation.y );
    observed_state.push_back( orientation.z );

    const int velocity = roverModel->getVelocityState();
    observed_state.push_back( velocity );

    const int steering = roverModel->getSteeringState();
    observed_state.push_back( steering );

    printState( observed_state );

    return observed_state;
}


void DRLPlugin::printState( const vector<float> &observed_state )
{
    vector<float>::const_iterator it;
    stringstream stream;
    stream << "State = ( ";
    for(it = observed_state.begin(); it != observed_state.end(); ++it)
        stream << setprecision(2) << *it << " ";
    stream << ")";

    gzmsg << stream.str() << endl;
}


void DRLPlugin::trainAlgorithm()
{
    bool collision = roverModel->checkCollision();
    if( collision ){
        gzmsg << "Collision detected !!!" << endl;
        const float bad_reward = -1000;
        rlAgent->updateQValues( bad_reward );
        // Reset gazebo model to initial position
        gzmsg << "Reseting model to initial position." << endl;
        roverModel->resetModel();
    }

    common::Time elapsedTime = actionTimer.GetElapsed();
    if( elapsedTime >= actionInterval ){

        gzmsg << endl;
        gzmsg << "Step = " << numSteps << endl;
        unsigned char* image_data = const_cast<unsigned char*>(roverModel->getImage());
        if( image_data ){
            cv::Mat input_image = cv::Mat( roverModel->getImageHeight(),
                                           roverModel->getImageWidth(),
                                           CV_8UC3,
                                           image_data );

            input_image.convertTo(input_image, CV_32FC3);

            // Feed input state to network
            vector<float> observed_state = getState();
            const float *input_state = &observed_state[0];

            vector<float> qvalues = caffeNet->Predict( input_image, input_state );

            const float reward = roverModel->getReward( setPoint );
            const unsigned state_index = rlAgent->fetchState( observed_state, qvalues );

            roverModel->saveImage( state_index );
            rlAgent->updateQValues( reward, state_index );

            // Terminal state
            if( reward > -0.2 )
                roverModel->resetModel();

            const unsigned action = rlAgent->chooseAction( state_index, true );
            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;
        }

        ++numSteps;
        // Terminate simulation after maxStep
        if( numSteps == maxSteps){
            rlAgent->savePolicy();

            gzmsg << endl;
            gzmsg << "Simulation reached max number of steps." << endl;
            gzmsg << "Terminating simulation..." << endl;
            msgs::ServerControl server_msg;
            server_msg.set_stop(true);
            serverControlPub->Publish(server_msg);
        }

        actionTimer.Reset();
        actionTimer.Start();
    }
}


void DRLPlugin::runAlgorithm()
{
    common::Time elapsedTime = actionTimer.GetElapsed();
    if( elapsedTime >= actionInterval ){

        // Feed input image to network
        unsigned char* image_data = const_cast<unsigned char*>(roverModel->getImage());
        if( image_data ){
            cv::Mat input_image = cv::Mat( roverModel->getImageHeight(),
                                           roverModel->getImageWidth(),
                                           CV_8UC3,
                                           image_data );

            input_image.convertTo(input_image, CV_32FC3);

            // Feed input state to network
            vector<float> observed_state = getState();
            const float *input_state = &observed_state[0];

            vector<float> qvalues = caffeNet->Predict( input_image, input_state );
            vector<float>::iterator qvalues_it = max_element( qvalues.begin(), qvalues.end() );
            const unsigned action = distance( qvalues.begin(), qvalues_it );

            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;

            ++numSteps;
        }

        // Terminate simulation after maxStep
        if( numSteps == maxSteps){
            gzmsg << endl;
            gzmsg << "Simulation reached max number of steps." << endl;
            gzmsg << "Terminating simulation..." << endl;
            msgs::ServerControl server_msg;
            server_msg.set_stop(true);
            serverControlPub->Publish(server_msg);
        }

        actionTimer.Reset();
        actionTimer.Start();
    }
}
