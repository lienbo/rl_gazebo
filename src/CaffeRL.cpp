#include "CaffeRL.hpp"
#include <caffe/util/hdf5.hpp>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

using cv::Mat;

using caffe::Caffe;
using caffe::Blob;
using caffe::Net;
using caffe::BlobProto;
using caffe::NetParameter;
using caffe::LayerParameter;
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


CaffeRL::CaffeRL( const string &solver_file, const string &weights_file ):
    imageBlobName("images"), stateBlobName("states"), outputBlobName("output"),
    targetBlobName("target"), iteration(0)
{
    Caffe::set_mode(Caffe::GPU);

    caffe::ReadProtoFromTextFileOrDie( solver_file, &solverParam );

    caffeSolver.reset( caffe::SolverRegistry<float>::CreateSolver( solverParam ) );
    caffeNet = caffeSolver->net();

    tempcaffeSolver.reset( caffe::SolverRegistry<float>::CreateSolver( solverParam ) );
    tempcaffeNet = tempcaffeSolver->net();

//    caffeNet.reset( new Net<float>(network_file, caffe::TEST) );
    if( access( weights_file.c_str(), F_OK ) != -1  ){
        cout << "Loading network weights..." << endl;
        caffeNet->CopyTrainedLayersFrom( weights_file );
        tempcaffeNet->CopyTrainedLayersFrom( weights_file );
    }
}


void CaffeRL::getTransformation( const string &network_file )
{
    NetParameter net_parameters;
    ReadProtoFromTextFileOrDie( network_file, &net_parameters );
    LayerParameter layer_parameters = net_parameters.layer(0);
    if( layer_parameters.has_transform_param() ){
        TransformationParameter transformation_params = layer_parameters.transform_param();
        dataTransformer.reset( new DataTransformer<float>( transformation_params, caffe::TEST ) );
    }
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


void CaffeRL::printBlobInfo( const string &blob_name, const Blob<float> &blob ) const
{
    cout << "\nBlob: " << blob_name << endl;
    cout << "Number of axis = " << blob.num_axes() << endl;
    cout << "Elements count = " << blob.count() << endl;
    for( size_t i = 0; i < blob.num_axes(); ++i )
        cout << "Axis = "<< i << " num elements = " << blob.shape(i) << endl;

    cout << endl;
}


void CaffeRL::printImageInfo( const Mat &image ) const
{
    cout << "Depth = " << image.depth() << endl;
    cout << "Number of channels = " << image.channels() << endl;
    cout << "Number of rows = " << image.rows << endl;
    cout << "Number of collumns = " << image.cols << endl;
}


void CaffeRL::printBlobContent( const string &blob_name, Blob<float> &blob ) const
{
    cout << "Blob: " << blob_name << endl;
    const float *content = blob.cpu_data();
    for( size_t i = 0; i < blob.count(); ++i )
        cout << i << " = " << content[i] << endl;
}


void CaffeRL::printOutput() const
{
    boost::shared_ptr<Blob<float> > output_blob = caffeNet->blob_by_name( outputBlobName );
    printBlobInfo( outputBlobName, *output_blob );
    printBlobContent( outputBlobName, *output_blob );
}


void CaffeRL::loadImageMean( const string &mean_file )
{
    BlobProto mean_proto;
    ReadProtoFromBinaryFileOrDie( mean_file.c_str(), &mean_proto );

    Blob<float> mean_blob;
    mean_blob.FromProto(mean_proto);

    imageMean.CopyFrom(mean_blob, false, true);
}


// Load hdf5 files with ONLY ONE GROUP
void CaffeRL::loadHDF5( const string &file_name, unsigned max_dim )
{
    Blob<float> output_blob;

    hid_t file_hid = H5Fopen( file_name.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    CHECK_GE(file_hid, 0) << "Couldn't open " << file_name;

    string source_layer_name = caffe::hdf5_get_name_by_idx(file_hid, 0);
    cout << "source_layer_name = " << source_layer_name << endl;

    hdf5_load_nd_dataset( file_hid, source_layer_name.c_str(), 0, max_dim, &output_blob);

    printBlobInfo( source_layer_name, output_blob );

    H5Fclose(file_hid);
}


void CaffeRL::feedImage( const Mat &input_image )
{
    // Remove mean
    boost::shared_ptr<Blob<float> > image_blob = caffeNet->blob_by_name( imageBlobName );

//    dataTransformer->Transform( input_image, image_blob.get() );

    Mat image(image_blob->shape(2), image_blob->shape(3), CV_32FC3, image_blob->mutable_cpu_data() );
    image = input_image / 256.0f;
}


void CaffeRL::feedState( const float *input_state )
{
//    const unsigned num_elements = sizeof( input_state ) / sizeof( float );
    boost::shared_ptr<Blob<float> > state_blob = caffeNet->blob_by_name( stateBlobName );

    // Load state blob with data;
    // The gpu memory is synced with the cpu memory -> verify if it is automatic
    float *states = state_blob->mutable_cpu_data();
    const unsigned input_size = state_blob->count();
    copy( input_state, input_state + input_size, states );
}


vector<float> CaffeRL::getOutput() const
{
    caffeNet->Forward();

    boost::shared_ptr<Blob<float> > output_blob = caffeNet->blob_by_name( outputBlobName );
    const float *output_data = output_blob->cpu_data();

    vector<float> output;
    const unsigned output_size = output_blob->count();
    for(size_t i = 0; i < output_size; ++i){
        output.push_back( output_data[i] );
    }

    return output;
}


void CaffeRL::backPropagation( const float &error, const unsigned &action, const unsigned &num_actions )
{
  // The error is the same of all actions
//    const float error_array[num_actions] = { error };

    // The output error for actions not selected are keep equal to zero
    float error_array[num_actions] = {0};
    for( size_t i = 0; i < num_actions; ++i ){
        if( i == action )
            error_array[i] = error;
    }

    boost::shared_ptr<Blob<float> > output_blob = tempcaffeNet->blob_by_name( outputBlobName );
    float *output_diff = output_blob->mutable_cpu_diff();
    copy( error_array, error_array + num_actions, output_diff );
    tempcaffeNet->Backward();
}


void CaffeRL::saveModelWeights( const unsigned &iteration) const
{
    string model_filename = solverParam.snapshot_prefix() + "_iter_" +
            to_string(iteration) + ".caffemodel";
//    LOG(INFO) << "Snapshotting to binary proto file " << model_filename;

    NetParameter net_param;
    tempcaffeNet->ToProto(&net_param, solverParam.snapshot_diff());
    WriteProtoToBinaryFile(net_param, model_filename);
}


bool CaffeRL::abs_compare(float a, float b)
{
    return (abs(a) < abs(b));
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

        // QMax = optimal future value
        const float QMax = *max_element( observed_state_output.begin(), observed_state_output.end() );
        // Scale QTarget to [-1,1]
        const float abs_value = abs( *max_element( previous_state_output.begin(),
                previous_state_output.end(), abs_compare) );
        if( abs_value != 0 ){
            // QTarget = learned value
            const float QTarget = (reward + gamma * QMax) / abs_value;

            // Acumulated squared error (acc_err)
            // accumulated_error += alpha * ( QTarget - previous_state_output[action] );
            const float error = ( QTarget - previous_state_output[action] );

            // Update network weights
            backPropagation( error, action );

            ++iteration;
            cout << endl;
            cout << "Iteration = " << iteration << endl;
            cout << "Action = " << action << endl;
            cout << "Reward = " << reward << endl;
            cout << "QMax = " << QMax << endl;
            cout << "QTarget = " << QTarget << endl;
            cout << "Error = " << error << endl;
        }

        if( (iteration % solverParam.snapshot()) == 0 )
            saveModelWeights( iteration );
    }

    // caffeNet is used to run the inferences while tempcaffeNet run the backpropagation
    NetParameter params;
    tempcaffeNet->ToProto( &params );
    caffeNet->CopyTrainedLayersFrom( params );
}


vector<float> CaffeRL::Predict( const vector<float> &input_state )
{
    const float *input_state_ptr = &input_state[0];
    feedState( input_state_ptr );

    return getOutput();
}


vector<float> CaffeRL::Predict( cv::Mat input_image, const float *input_state )
{
    feedImage( input_image );
//    feedState( input_state );

    caffeNet->Forward();

    boost::shared_ptr<Blob<float> > output_blob = caffeNet->blob_by_name( outputBlobName );

    vector<float> output;
    const float *output_data = output_blob->cpu_data();
    for(size_t i = 0; i < output_blob->shape(1); ++i){
        output.push_back( output_data[i] );
    }

    return output;
}
