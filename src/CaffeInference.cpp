#include "CaffeInference.hpp"
#include <caffe/common.hpp>

using namespace std;
using namespace caffe;

CaffeInference::CaffeInference( const string &model_file,
                                const string &trained_file,
                                const string &mean_file )
{
    Caffe::set_mode(Caffe::GPU);

    caffeNet.reset(new Net<float>( model_file, TEST ));
    caffeNet->CopyTrainedLayersFrom(trained_file);
}
