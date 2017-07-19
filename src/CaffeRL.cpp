#include "CaffeRL.hpp"
#include <caffe/util/hdf5.hpp>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

using cv::Mat;
using caffe::Caffe;
using caffe::Net;
using caffe::NetParameter;
using caffe::TransformationParameter;
using caffe::DataTransformer;


Transition::Transition( const vector<float> &previous_state, const unsigned &action,
                        const float &reward, const vector<float> &observed_state)
{
    this->previousState = previous_state;
    this->action = action;
    this->reward = reward;
    this->observedState = observed_state;
}


CaffeRL::CaffeRL( const string &solver_file, const string &weights_file,
            const bool &train ):
    imageBlobName("images"), stateBlobName("states"), outputBlobName("output"),
    targetBlobName("target"), iteration(0)
{
    Caffe::set_mode(Caffe::GPU);

    caffe::ReadProtoFromTextFileOrDie( solver_file, &solverParam );

    caffeSolver.reset( caffe::SolverRegistry<float>::CreateSolver(solverParam));
    caffeNet = caffeSolver->net();
    trainCaffeNet.reset( new Net<float>( solverParam.net(), caffe::TRAIN) );

    if( !train ){
        caffeNet.reset( new Net<float>( solverParam.net(), caffe::TEST) );
        trainCaffeNet.reset( new Net<float>( solverParam.net(), caffe::TEST) );
    }

    if( access( weights_file.c_str(), F_OK ) != -1  ){
        cout << "Loading network weights: "<< weights_file << endl;
        caffeNet->CopyTrainedLayersFrom( weights_file );
        trainCaffeNet->CopyTrainedLayersFrom( weights_file );
    }

    BlobPtr blob_ptr = caffeNet->blob_by_name( outputBlobName );
    numActions = blob_ptr->count();
}


void CaffeRL::getTransformation( const string &network_file )
{
    NetParameter net_parameters;
    ReadProtoFromTextFileOrDie( network_file, &net_parameters );
    caffe::LayerParameter layer_parameters = net_parameters.layer(0);
    if( layer_parameters.has_transform_param() ){
        TransformationParameter transformation_params = layer_parameters.transform_param();
        dataTransformer.reset( new DataTransformer<float>( transformation_params, caffe::TEST ) );
    }
}


void CaffeRL::loadImageMean( const string &mean_file )
{
    caffe::BlobProto mean_proto;
    ReadProtoFromBinaryFileOrDie( mean_file.c_str(), &mean_proto );

    caffe::Blob<float> mean_blob;
    mean_blob.FromProto(mean_proto);

    imageMean.CopyFrom(mean_blob, false, true);
}


// Load hdf5 files with ONLY ONE GROUP
void CaffeRL::loadHDF5( const string &file_name, unsigned max_dim )
{
    caffe::Blob<float> output_blob;

    hid_t file_hid = H5Fopen( file_name.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    CHECK_GE(file_hid, 0) << "Couldn't open " << file_name;

    string source_layer_name = caffe::hdf5_get_name_by_idx(file_hid, 0);
    cout << "source_layer_name = " << source_layer_name << endl;

    hdf5_load_nd_dataset( file_hid, source_layer_name.c_str(), 0, max_dim, &output_blob);

    printBlobInfo( source_layer_name );

    H5Fclose(file_hid);
}


void CaffeRL::printNetInfo() const
{
    cout << "\nNetwork name = " << caffeNet->name() << endl;
    cout << endl;

    vector<string> layers = caffeNet->layer_names();
    cout << "Number of layers = " << layers.size() << endl;
    for(vector<string>::iterator it = layers.begin(); it != layers.end(); ++it)
        cout << "Layer name = " << *it << endl;
    cout << endl;

    vector<string> blobs = caffeNet->blob_names();
    cout << "Number of blobs = " << blobs.size() << endl;
    for(vector<string>::iterator it = blobs.begin(); it != blobs.end(); ++it)
        cout << "Blob name = " << *it << endl;
    cout << endl;
}


void CaffeRL::printBlobInfo( const string &blob_name ) const
{
    BlobPtr blob = caffeNet->blob_by_name( blob_name );

    cout << "\nBlob: " << blob_name << endl;
    cout << "Number of axis = " << blob->num_axes() << endl;
    cout << "Elements count = " << blob->count() << endl;
    for( size_t i = 0; i < blob->num_axes(); ++i )
        cout << "Axis = "<< i << " num elements = " << blob->shape(i) << endl;

    cout << endl;
}


void CaffeRL::printBlobContent( const string &blob_name ) const
{
    BlobPtr blob = caffeNet->blob_by_name( blob_name );

    cout << "Blob: " << blob_name << endl;
    const float *content = blob->cpu_data();
    for( size_t i = 0; i < blob->count(); ++i )
        cout << i << " = " << content[i] << endl;
}


void CaffeRL::printImageInfo( const Mat &image ) const
{
    cout << "Depth = " << image.depth() << endl;
    cout << "Number of channels = " << image.channels() << endl;
    cout << "Number of rows = " << image.rows << endl;
    cout << "Number of collumns = " << image.cols << endl;
}


void CaffeRL::feedBlob( const float *input_data, const string &blob_name )
{
    BlobPtr blob_ptr = caffeNet->blob_by_name( blob_name );

    // Load blob with input_data;
    // The gpu memory is synced with the cpu memory
    float *blob_content = blob_ptr->mutable_cpu_data();
    const unsigned input_size = blob_ptr->count();

    copy( input_data, input_data + input_size, blob_content );
}


vector<float> CaffeRL::getBlobContent( const string &blob_name ) const
{
    BlobPtr output_blob = caffeNet->blob_by_name( blob_name );
    const float *output_data = output_blob->cpu_data();

    vector<float> output;
    const unsigned output_size = output_blob->count();
    for(size_t i = 0; i < output_size; ++i){
        output.push_back( output_data[i] );
    }

    return output;
}


void CaffeRL::backPropagation( const float &error, const unsigned &action )
{
  // The error is the same of all actions
//    const float error_array[numActions] = { error };

    // The output error for actions not selected are keep equal to zero
    float error_array[numActions] = {0};
    for( size_t i = 0; i < numActions; ++i ){
        if( i == action )
            error_array[i] = error;
    }

    BlobPtr output_blob = trainCaffeNet->blob_by_name( outputBlobName );
    float *output_diff = output_blob->mutable_cpu_diff();
    copy( error_array, error_array + numActions, output_diff );
    trainCaffeNet->Backward();
}


void CaffeRL::saveModelWeights( const unsigned &iteration) const
{
    string model_filename = solverParam.snapshot_prefix() + "_iter_" +
            to_string(iteration) + ".caffemodel";
//    LOG(INFO) << "Snapshotting to binary proto file " << model_filename;

    NetParameter net_param;
    trainCaffeNet->ToProto(&net_param, solverParam.snapshot_diff());
    WriteProtoToBinaryFile(net_param, model_filename);
}


void CaffeRL::Train( const vector<Transition> &transitions_container )
{
    // Learning rate -> this value is replaced by the Network learning rate
    // Discount factor
    const float gamma = 0.8;

    const unsigned batch_size = transitions_container.size();

    cout << "\nTraining network... " << endl;
    cout << "Batch size = " << batch_size << endl;

    // float accumulated_error = 0;
    for( size_t i = 0; i < batch_size; ++i ){
        vector<float> previous_state = transitions_container[i].previousState;
        vector<float> observed_state = transitions_container[i].observedState;
        const unsigned action = transitions_container[i].action;
        const float reward = transitions_container[i].reward;

        // The NN calculates the Q-Values for each state
        vector<float> previous_state_output = Predict( previous_state );
        vector<float> observed_state_output = Predict( observed_state );

        // QMax = optimal future value (DQN)
//        const float QMax = *max_element( observed_state_output.begin(),
//                                         observed_state_output.end() );

        // Calculate QMax according:
        // Deep Reinforcement Learning with Double Q-learning
        vector<float> temp_observed_output = Predict(trainCaffeNet, observed_state);
        vector<float>::iterator qvalues_it = max_element( temp_observed_output.begin(),
                 temp_observed_output.end() );
        const unsigned max_action = distance( temp_observed_output.begin(),
                  qvalues_it );
        const float QMax = observed_state_output[max_action];


        // QTarget = learned value
        const float QTarget = (reward + gamma * QMax);

        // Acumulated squared error (acc_err)
        // accumulated_error += alpha * ( QTarget - previous_state_output[action] );
        float error = ( QTarget - previous_state_output[action] );

//        // Clip error to [-1,1]
//        if( error > 1 ){
//            error = 1;
//        }else if( error < -1 ){
//            error = -1;
//        }

        // Update network weights
        backPropagation( error, action );

        ++iteration;
        cout << endl;
        cout << "Iteration = " << iteration << endl;
        cout << "Action = " << action << endl;
        cout << "Reward = " << reward << endl;
        cout << "Output = " << previous_state_output[action] << endl;
        cout << "QTarget = " << QTarget << endl;
        cout << "Error = " << error << endl;
        printBlobContent( "states" );
        printBlobContent( "output" );

        if( (iteration % solverParam.snapshot()) == 0 )
            saveModelWeights( iteration );
    }

    // caffeNet is used to run the inferences
    // trainCaffeNet run the backpropagation
    NetParameter params;
    trainCaffeNet->ToProto( &params );
    caffeNet->CopyTrainedLayersFrom( params );
}


void CaffeRL::TrainSoftmax( const vector<Transition> &transitions_container )
{
    // Learning rate -> this value is replaced by the Network learning rate
    // Discount factor
    const float gamma = 0.9;

    const unsigned batch_size = transitions_container.size();

    cout << "\nTraining network... " << endl;
    cout << "Batch size = " << batch_size << endl;

    // float accumulated_error = 0;
    // This is not reinforcement learning -> Not using observed_state or reward
    for( size_t i = 0; i < batch_size; ++i ){
        vector<float> previous_state = transitions_container[i].previousState;
        vector<float> observed_state = transitions_container[i].observedState;
        const float action = (float) transitions_container[i].action;
        const float reward = transitions_container[i].reward;

        const float *previous_state_ptr = &previous_state[0];
        feedBlob( previous_state_ptr, stateBlobName );

        const float *target = &action;
        feedBlob( target, targetBlobName );

        caffeSolver->Step(1);

        cout << endl;
        cout << "Iteration = " << iteration << endl;
        cout << "Action = " << action << endl;
        printBlobContent( "states" );
        printBlobContent( "output" );
        printBlobContent( "loss" );

        ++iteration;
    }
}


vector<float> CaffeRL::Predict( const vector<float> &input_state )
{
    const float *input_state_ptr = &input_state[0];
    feedBlob( input_state_ptr, stateBlobName );
    caffeNet->Forward();

    return getBlobContent( outputBlobName );
}


vector<float> CaffeRL::Predict( boost::shared_ptr<caffe::Net<float> > network,
                                const vector<float> &input_state )
{
    const float *input_state_ptr = &input_state[0];
    feedBlob( input_state_ptr, stateBlobName );
    network->Forward();

    return getBlobContent( outputBlobName );
}


vector<float> CaffeRL::Predict( cv::Mat input_image, const float *input_state )
{
//    feedBlob( input_state, stateBlobName );
//    feedBlob( input_image, imageBlobName );

    caffeNet->Forward();

    BlobPtr output_blob = caffeNet->blob_by_name(outputBlobName);

    vector<float> output;
    const float *output_data = output_blob->cpu_data();
    for(size_t i = 0; i < output_blob->shape(1); ++i){
        output.push_back( output_data[i] );
    }

    return output;
}
