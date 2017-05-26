#include <boost/make_shared.hpp>
#include "DRLPlugin.hpp"

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace gazebo;


DRLPlugin::DRLPlugin() : numSteps(0), trainNet(true)
{
    initialPos.push_back( math::Pose(0, 0, .12, 0, 0, 0) );
    initialPos.push_back( math::Pose(0, 4, .12, 0, 0, -1.5708) );
    initialPos.push_back( math::Pose(-5.5, -3.3, .12, 0, 0, 1.5708) );
    initialPos.push_back( math::Pose(4, 0, .12, 0, 0, 3.1416) );

    destinationPos.push_back( math::Vector3(2, 0, 0.1) );
    destinationPos.push_back( math::Vector3(-5, 0, 0.1) );
    destinationPos.push_back( math::Vector3(-1.5, -3.5, 0.1) );
    destinationPos.push_back( math::Vector3(6, 6, 0.1) );
}

DRLPlugin::~DRLPlugin() {}

void DRLPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{
    maxSteps = 10000;
    transport::NodePtr node( new transport::Node() );
    node->Init();
    serverControlPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");

    roverModel = boost::make_shared<RoverModel>( model, sdf );
    rlAgent = boost::make_shared<QLearner>( roverModel->getNumActions() );

    const string model_file = "./caffe/network/drl_gazebo.prototxt";
    string weights_file = "./caffe/models/drl_gazebo_iter_1000.caffemodel";
    if( sdf->HasElement( "weights_file" ) ){
        weights_file = sdf->Get<string>("weights_file");
    }

    string solver_file = "./caffe/network/nfq_gazebo_solver.prototxt";
    if( sdf->HasElement( "solver_file" ) ){
        solver_file = sdf->Get<string>("solver_file");
    }

    caffeNet = boost::make_shared<CaffeRL>( model_file, weights_file, solver_file );

    if( sdf->HasElement( "train" ) )
        trainNet = sdf->Get<bool>("train");

    if( sdf->HasElement( "max_steps" ) )
        maxSteps = sdf->Get<unsigned>("max_steps");

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DRLPlugin::onUpdate, this, _1));

    // World simulation time will be used to synchronize the actions
    worldPtr = model->GetWorld();
    timeMark = worldPtr->GetSimTime();

    // Apply first action
    vector<float> observed_state = roverModel->getState();
    const unsigned state_index = rlAgent->fetchState( observed_state );
    const unsigned action = rlAgent->chooseAction( state_index, true );
    roverModel->applyAction( action );
    gzmsg << "Applying action = " << action << endl;
}


void DRLPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();
    trainNet ? trainAlgorithm() : testAlgorithm();
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

    common::Time elapsedTime = worldPtr->GetSimTime() - timeMark;
    if( elapsedTime >= roverModel->getActionInterval() ){
        gzmsg << endl;
        gzmsg << "Step = " << numSteps << endl;
        unsigned char* image_data = const_cast<unsigned char*>(roverModel->getImage());
        if( image_data ){
            // Terminal state
            if( roverModel->isTerminalState() ){
                gzmsg << "Model reached terminal state !!!" << endl;
                const float good_reward = 10000;
                rlAgent->updateQValues( good_reward );
                roverModel->resetModel();
            }

            cv::Mat input_image = cv::Mat( roverModel->getImageHeight(),
                                           roverModel->getImageWidth(),
                                           CV_8UC3,
                                           image_data );

            input_image.convertTo(input_image, CV_32FC3);

            // Feed input state to network
            vector<float> observed_state = roverModel->getState();
            const float *input_state = &observed_state[0];

            vector<float> qvalues = caffeNet->Predict( input_image, input_state );

            const float reward = roverModel->getReward();
            const unsigned state_index = rlAgent->fetchState( observed_state, qvalues );

            roverModel->saveImage( state_index );
            rlAgent->updateQValues( reward, state_index );

            const unsigned action = rlAgent->chooseAction( state_index, true );
            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;
        }

        roverModel->endStep();
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

        timeMark = worldPtr->GetSimTime();
    }
}


void DRLPlugin::testAlgorithm()
{
    common::Time elapsedTime = worldPtr->GetSimTime() - timeMark;
    if( elapsedTime >= roverModel->getActionInterval() ){

        // Feed input image to network
        unsigned char* image_data = const_cast<unsigned char*>(roverModel->getImage());
        if( image_data ){
            if( roverModel->isTerminalState() ){
                gzmsg << "Model reached terminal state !!!" << endl;
                roverModel->resetModel();
            }

            cv::Mat input_image = cv::Mat( roverModel->getImageHeight(),
                                           roverModel->getImageWidth(),
                                           CV_8UC3,
                                           image_data );

            input_image.convertTo(input_image, CV_32FC3);

            // Feed input state to network
            vector<float> observed_state = roverModel->getState();
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

        timeMark = worldPtr->GetSimTime();
    }
}
