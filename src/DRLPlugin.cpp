#include <gazebo/msgs/server_control.pb.h>
#include <boost/make_shared.hpp>
#include "DRLPlugin.hpp"

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace gazebo;


DRLPlugin::DRLPlugin() : numSteps(0)
{
    actionTimer.Reset();
    actionInterval.Set(1,0);
    setPoint.Set(2, 0, 0.1);
}

DRLPlugin::~DRLPlugin() {}

void DRLPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    maxSteps = 20000;
    transport::NodePtr node( new transport::Node() );
    node->Init();
    serverControlPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");

    const unsigned num_actions = 6;
    roverModel = boost::make_shared<RoverModel>( model, sdf );

    const string model_file = "./caffe/network/drl_gazebo.prototxt";
    const string trained_file = "./caffe/models/drl_gazebo_iter_10000.caffemodel";
    caffeNet = boost::make_shared<CaffeInference>( model_file, trained_file );

    if( sdf->HasElement( "max_steps" ) )
        maxSteps = sdf->Get<unsigned>("max_steps");

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DRLPlugin::onUpdate, this, _1));

    actionTimer.Start();
    const unsigned action = 0;
}


void DRLPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();
    runAlgorithm();
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
    observed_state.push_back( orientation.x );
    observed_state.push_back( orientation.y );
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
    gzmsg << endl;
    stringstream stream;
    stream << "State = ( ";
    for(it = observed_state.begin(); it != observed_state.end(); ++it)
        stream << setprecision(2) << *it << " ";
    stream << ")";

    gzmsg << stream.str() << endl;
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

            vector<float> policy = caffeNet->Predict( input_image, input_state );
            vector<float>::iterator policy_it = max_element( policy.begin(), policy.end() );
            const unsigned action = distance( policy.begin(), policy_it );

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
