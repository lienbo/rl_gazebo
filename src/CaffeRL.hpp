#ifndef CAFFE_RL_HPP_
#define CAFFE_RL_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <string>

#include <caffe/data_transformer.hpp>
#include <caffe/caffe.hpp>


struct Transition
{
    Transition( const std::vector<float> &previous_state, const unsigned &action,
                const float &reward, const std::vector<float> &observed_state);

    std::vector<float> previousState;
    unsigned action;
    float reward;
    std::vector<float> observedState;
};


class CaffeRL
{
    public:
    CaffeRL( const std::string &solver_file, const std::string &weights_file,
            const bool &train );

    void getTransformation( const std::string &model_file );
    void loadImageMean( const std::string &mean_file );
    void loadHDF5( const std::string &file_name, unsigned max_dim );

    void printNetInfo() const;
    void printBlobInfo( const std::string &blob_name ) const;
    void printBlobContent( const std::string &blob_name ) const;
    void printImageInfo( const cv::Mat &image ) const;

    void Train( const std::vector<Transition> &transitions_container );
    void TrainSoftmax( const std::vector<Transition> &transitions_container );

    std::vector<float> Predict( const std::vector<float> &input_state );
    std::vector<float> Predict( boost::shared_ptr<caffe::Net<float> > network,
                                const std::vector<float> &input_state );
    std::vector<float> Predict( cv::Mat input_image, const float *input_state);

    private:
    typedef boost::shared_ptr<caffe::Blob<float> > BlobPtr;

    void feedBlob( const float *input_data, const std::string &blob_name );
    std::vector<float> getBlobContent( const std::string &blob_name ) const;
    void backPropagation( const float &error, const unsigned &action );
    void saveModelWeights( const unsigned &iteration ) const;

    // Target network
    boost::shared_ptr<caffe::Net<float> > caffeNet;
    // Create temporary network to run the backpropagation ( online network )
    boost::shared_ptr<caffe::Net<float> > trainCaffeNet;
    boost::shared_ptr<caffe::Solver<float> > caffeSolver;
    boost::shared_ptr<caffe::DataTransformer<float> > dataTransformer;
    caffe::SolverParameter solverParam;
    caffe::Blob<float> imageMean;

    const std::string imageBlobName, stateBlobName, outputBlobName, targetBlobName;
    unsigned iteration, numActions;
};

#endif
