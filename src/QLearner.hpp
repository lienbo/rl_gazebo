#ifndef QLEARNER_HPP_
#define QLEARNER_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <vector>
#include <random>
#include "State.hpp"


class QLearner{
    public:
    QLearner( const unsigned &num_actions );
    ~QLearner();

    typedef std::vector<State> StatesContainer;

    const unsigned fetchState( const std::vector<float> &observed_state );
    const unsigned fetchState( const std::vector<float> &observed_state, std::vector<float> qvalues );
    const unsigned chooseAction( const unsigned &state_index, const float &probability = 0.3 );
    const unsigned selectAction( const unsigned &state_index );
    const unsigned updateAction( const unsigned &state_index, unsigned action, const float &probability = 0.3 );
    void updateQValues( const float& reward, const unsigned &state_index);
    void updateQValues( const float& reward );
    void printQValues( const std::string &message, const unsigned &current_index ) const;
    void loadPolicy();
    void savePolicy( bool save_converged = false, bool standardize = false );

    private:
    float alpha, gamma;
    unsigned numActions;
    unsigned lastIndex;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> uniformDist;
    std::string outputDir;

    // There may be an infinity number of states, thus states must be stored in dynamic vectors
    StatesContainer qlearnerStates;
};

#endif
