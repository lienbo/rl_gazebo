#ifndef CAFFE_INFERENCE_HPP_
#define CAFFE_INFERENCE_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <boost/shared_ptr.hpp>
#include <string>

#include <caffe/data_transformer.hpp>
#include <caffe/caffe.hpp>


class CaffeInference{
    public:
    CaffeInference( const std::string &model_file,
                    const std::string &trained_file );

    void getTransformation( const std::string &model_file );

    void loadHDF5( const std::string &file_name, unsigned max_dim );
    void loadImageMean( const std::string &mean_file );
    std::vector<float> Predict( cv::Mat input_image, const float *input_state);

    private:
    boost::shared_ptr<caffe::Net<float> > caffeNet;
    boost::shared_ptr<caffe::DataTransformer<float> > dataTransformer;
    caffe::Blob<float> imageMean;
    const std::string imageBlobName, stateBlobName, outputBlobName;

    void feedImage( const cv::Mat &input_image );
    void feedState( const float *input_state );

    void printOutput() const;
    void printNetInfo() const;
    void printImageInfo( const cv::Mat &image ) const;
    void printBlobInfo( const std::string &blob_name, const caffe::Blob<float> &blob ) const;
    void printBlobContent( const std::string &blob_name, caffe::Blob<float> &blob ) const;
};

#endif
