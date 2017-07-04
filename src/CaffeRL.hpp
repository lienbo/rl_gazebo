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
    CaffeRL( const std::string &solver_file, const std::string &trained_file );

    void getTransformation( const std::string &model_file );

    void loadHDF5( const std::string &file_name, unsigned max_dim );
    void loadImageMean( const std::string &mean_file );
    std::vector<float> Predict( const std::vector<float> &input_state );
    std::vector<float> Predict( cv::Mat input_image, const float *input_state);

    void printOutput() const;
    void printNetInfo() const;
    void printImageInfo( const cv::Mat &image ) const;
    void printBlobInfo( const std::string &blob_name, const caffe::Blob<float> &blob ) const;
    void printBlobContent( const std::string &blob_name, caffe::Blob<float> &blob ) const;

    void Train( const std::vector<Transition> &transitions_container );

    private:
    void feedImage( const cv::Mat &input_image );
    void feedState( const float *input_state );
    std::vector<float> getOutput() const;
    void backPropagation( const float &error, const unsigned &action, const
            unsigned &num_actions = 5 );
    void saveModelWeights( const unsigned &iteration ) const;
    static bool abs_compare(float a, float b);

    boost::shared_ptr<caffe::Net<float> > caffeNet;
    // Create temporary network to run the backpropagation
    boost::shared_ptr<caffe::Net<float> > tempcaffeNet;
    boost::shared_ptr<caffe::Solver<float> > caffeSolver;
    boost::shared_ptr<caffe::Solver<float> > tempcaffeSolver;
    caffe::SolverParameter solverParam;
//    boost::shared_ptr<caffe::SolverParameter> solverParam;

    boost::shared_ptr<caffe::DataTransformer<float> > dataTransformer;
    caffe::Blob<float> imageMean;
    const std::string imageBlobName, stateBlobName, outputBlobName, targetBlobName;
    unsigned iteration;
};

#endif
