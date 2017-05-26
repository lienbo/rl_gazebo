#ifndef STATE_HPP_
#define STATE_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <vector>


struct State{
    State( unsigned num_actions, const std::vector<float> &observed_state );
    State( std::vector<float> qvalues, const std::vector<float> &state );
    ~State();

    unsigned action;
    float QValue, maxQValue;
    std::vector<float>::iterator action_it;
    std::vector<float> stateValues;
    std::vector<float> QValues;

    // This vector inform if there is no significant changes in QValues:
    // newQValue - Qvalue < convergenceTreshold
    // convergedState size == num actions == QValues size
    std::vector<bool> convergedState;
    float convergenceTreshold;

    bool compareState( const std::vector<float> &observed_state );
};

#endif
