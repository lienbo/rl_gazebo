#ifndef CAFFE_INFERENCE_HPP_
#define CAFFE_INFERENCE_HPP_

#include <boost/shared_ptr.hpp>
#include <caffe/net.hpp>
#include <string>


class CaffeInference{
    public:
    CaffeInference( const std::string &model_file,
                   const std::string &trained_file,
                   const std::string &mean_file );

    private:
    boost::shared_ptr<caffe::Net<float> > caffeNet;

};

#endif
