#include "CaffeInference.hpp"
#include <caffe/common.hpp>

using namespace std;
using namespace caffe;

CaffeInference::CaffeInference( const string &model_file, const string &trained_file )
{
    Caffe::set_mode(Caffe::GPU);

    caffeNet.reset(new Net<float>( model_file, TEST ));
    caffeNet->CopyTrainedLayersFrom(trained_file);
}


void CaffeInference::printInfo() const
{
    cout << "Network name = " << caffeNet->name() << endl;

    vector<string> layers = caffeNet->layer_names();
    cout << "Number of layers = " << layers.size() << endl;
    for(vector<string>::iterator it = layers.begin(); it != layers.end(); ++it)
        cout << "Layer name = " << *it << endl;

    vector<string> blobs = caffeNet->blob_names();
    cout << "Number of blobs = " << blobs.size() << endl;
    for(vector<string>::iterator it = blobs.begin(); it != blobs.end(); ++it)
        cout << "Blob name = " << *it << endl;
}

