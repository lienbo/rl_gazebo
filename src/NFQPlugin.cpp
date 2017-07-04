#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include "NFQPlugin.hpp"
#include <random>

using namespace std;
using namespace gazebo;



NFQPlugin::NFQPlugin() : numSteps(0), maxSteps(5000), numTrials(0),
        maxTrials(1), train(true), memoryReplaySize(128), batchSize(32),
        outputDir("./caffe/weights/")
{
    destinationPos.push_back( math::Vector3(0, 0, 0) );

    initialPos.push_back( math::Pose(-4, -2, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-3, -1, 0, 0, 0, 0) );
    initialPos.push_back( math::Pose(-2, 0, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(-3, 1, 0, 0, 0, 0) );
    initialPos.push_back( math::Pose(-4, 2, 0, 0, 0, 0) );

//    initialPos.push_back( math::Pose(4, -2, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(2, 0, 0, 0, 0, 0) );
//    initialPos.push_back( math::Pose(4, 2, 0, 0, 0, 0) );

    boost::filesystem::path dir( outputDir.c_str() );
    boost::filesystem::create_directories( dir );
}


NFQPlugin::~NFQPlugin() {}


void NFQPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdfPtr )
{
    transport::NodePtr node( new transport::Node() );
    node->Init();
    serverControlPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");

    loadParameters( sdfPtr );

    roverModel = boost::make_shared<RoverModel>( model, sdfPtr );
    roverModel->setOriginAndDestination( initialPos, destinationPos );

    const unsigned num_actions = roverModel->getNumActions();
    uniform_int_distribution<int> temp_uniform_dist( 0, num_actions - 1 );
    uniformDist.param( temp_uniform_dist.param() );

    // onUpdate is called each simulation step.
    // It will be used to publish simulation data (sensors, pose, etc).
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind( &NFQPlugin::onUpdate, this, _1) );

    // World simulation time will be used to synchronize the actions
    worldPtr = model->GetWorld();
    timeMark = worldPtr->GetSimTime();

    // Apply first action
    roverModel->resetModel();
    firstAction();
}


void NFQPlugin::loadParameters( const sdf::ElementPtr &sdfPtr )
{
//    const string model_file = "./caffe/network/nfq_gazebo.prototxt";
    string solver_file = "./caffe/network/nfq_gazebo_solver.prototxt";
    if( sdfPtr->HasElement( "solver_file" ) ){
        solver_file = sdfPtr->Get<string>("solver_file");
    }

    string weights_file;
    if( sdfPtr->HasElement( "weights_file" ) ){
        weights_file = sdfPtr->Get<string>("weights_file");
    }

    caffeRL = boost::make_shared<CaffeRL>( solver_file, weights_file );

    if( sdfPtr->HasElement( "mode" ) ){
        string execution_mode = sdfPtr->Get<string>("mode");
        if( execution_mode == "train" ){
            train = true;
        }
        if( execution_mode == "test" ){
            train = false;
        }
    }

    if( sdfPtr->HasElement( "max_steps" ) )
        maxSteps = sdfPtr->Get<unsigned>("max_steps");

    if( sdfPtr->HasElement( "max_trials" ) )
        maxTrials = sdfPtr->Get<unsigned>("max_trials");
}


unsigned NFQPlugin::eGreedy( unsigned action, const float &probability )
{
    bernoulli_distribution bernoulli( probability );
    generator.seed(time(0));
    // Bernoulli distribution to change action
    if( bernoulli(generator) ){
        // Equal (uniform) probability to choose an action
        action = uniformDist(generator);
    }
    return action;
}


void NFQPlugin::onUpdate( const common::UpdateInfo &info )
{
    roverModel->velocityController();
    roverModel->steeringWheelController();
    train ? trainAlgorithm() : testAlgorithm();
}


// This function doesn't call updateQValues because the initial position doesn't
// have a last state to update.
void NFQPlugin::firstAction()
{
    vector<float> observed_state = roverModel->getState();

    unsigned action;
    if(train){
        action = roverModel->bestAction();
        const float initial_epsilon = 0.7; // Final epsilon is zero
        const float epsilon = initial_epsilon*(1.0 - ( round(10 * numSteps/maxSteps) /10 ));
        gzmsg << "Epsilon = " << epsilon << endl;
        action = eGreedy( action, epsilon );
    }else{
        vector<float> qvalues = caffeRL->Predict( observed_state );
        vector<float>::iterator qvalues_it = max_element( qvalues.begin(), qvalues.end() );
        const unsigned action = distance( qvalues.begin(), qvalues_it );
    }

    roverModel->applyAction( action );
    gzmsg << "Applying action = " << action << endl;
    previousAction = action;
    previousState = observed_state;
}


// Using memory replay, the training has two phases:
// 1 - Run the neural network inference moving the robot for n steps
// 2 - Train the NN model
void NFQPlugin::trainAlgorithm()
{
    bool collision = roverModel->checkCollision();
    if( collision ){
        gzmsg << "Collision detected !!!" << endl;
        const float bad_reward = -100;

        vector<float> observed_state = roverModel->getState();
        Transition transition( previousState, previousAction, bad_reward, observed_state );
        transitionsContainer.push_back( transition );

        // Reset gazebo model to initial position
        roverModel->resetModel();
        firstAction();
    }

    common::Time elapsed_time = worldPtr->GetSimTime() - timeMark;
    if( elapsed_time >= roverModel->getActionInterval() ){
        gzmsg << endl;
        gzmsg << "Trial-step = " << numTrials << "-" << numSteps << endl;

        // Terminal state
        if( roverModel->isTerminalState() ){
            gzmsg << "Model reached terminal state !!!" << endl;
            roverModel->resetModel();
            firstAction();

        }else if( roverModel->isDistancing() ){
            // Reset model if it is going in the opposite direction
            // This helps reduce the number of states decreasing the learning time
            gzmsg << "Wrong direction !" << endl;
            roverModel->resetModel();
            firstAction();

        }else{
            const float reward = roverModel->getReward();
            vector<float> observed_state = roverModel->getState();

            Transition transition( previousState, previousAction, reward, observed_state );
            transitionsContainer.push_back( transition );

//            vector<float> qvalues = caffeRL->Predict( observed_state );
            unsigned action = roverModel->bestAction();
            // E-greedy implementation
            const float initial_epsilon = 0.7; // Final epsilon is zero
            const float epsilon = initial_epsilon*(1.0 - ( round(10. * numSteps/maxSteps) /10. ));
            gzmsg << "Epsilon = " << epsilon << endl;
            action = eGreedy( action, epsilon );

            roverModel->applyAction( action );
            gzmsg << "Applying action = " << action << endl;

            previousAction = action;
            previousState = observed_state;
        }

        if( transitionsContainer.size() >= memoryReplaySize )
        {
            gzmsg << "Training Network..." << endl;
            // Randomly selects transition samples
            generator.seed(time(0));
            unsigned slice = floor( memoryReplaySize/batchSize );
            uniform_int_distribution<int> uniform_dist( 0, slice - 1 );
            vector<Transition> transitions_container;

            for( size_t i = 0; i < batchSize; ++i ){
                unsigned transition_it = uniform_dist( generator );
                transition_it += i * slice;
                transitions_container.push_back( transitionsContainer[ transition_it ] );
            }

            caffeRL->Train( transitions_container );
            gzmsg << "Finished training." << endl;

            // Delete transitions
            transitionsContainer.clear();
        }

        roverModel->endStep();
        ++numSteps;

        if( numSteps == maxSteps ){
            numSteps = 0;
            ++numTrials;
        }

        // Terminate simulation after maxTrials
        if( numTrials == maxTrials ){
            gzmsg << endl;
            gzmsg << "Simulation reached max number of trials." << endl;

            gzmsg << "Terminating simulation..." << endl;
            msgs::ServerControl server_msg;
            server_msg.set_stop(true);
            serverControlPub->Publish(server_msg);
        }

        timeMark = worldPtr->GetSimTime();
    }
}


void NFQPlugin::testAlgorithm()
{
    common::Time elapsed_time = worldPtr->GetSimTime() - timeMark;
    if( elapsed_time >= roverModel->getActionInterval() ){
        if( roverModel->isTerminalState() ){
            gzmsg << "Model reached terminal state !!!" << endl;
            roverModel->resetModel();
        }

        vector<float> observed_state = roverModel->getState();
        vector<float> qvalues = caffeRL->Predict( observed_state );
        vector<float>::iterator qvalues_it = max_element( qvalues.begin(), qvalues.end() );
        const unsigned action = distance( qvalues.begin(), qvalues_it );

        roverModel->applyAction( action );
        gzmsg << "Applying action = " << action << endl;
        gzmsg << endl;

        ++numSteps;
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
