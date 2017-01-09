#ifndef QLearner_HPP_
#define QLearner_HPP_

#include <vector>
#include <random>
#include <algorithm>


struct State{
    State( unsigned num_actions, const std::vector<float> &observed_state );
    ~State();

    unsigned action;
    float QValue, maxQValue;
    std::vector<float>::iterator action_it;
    std::vector<float> stateValues;
    std::vector<float> QValues;

    bool compareState( const std::vector<float> &observed_state );
};

class QLearner{
    public:
    QLearner( const unsigned &num_actions );
    ~QLearner();

    typedef std::vector<State> StatesContainer;
    const unsigned chooseAction( const std::vector<float> &observed_state, const bool &training = true );
    void updateQValues( const float& reward, const std::vector<float> &observed_state );

    private:
    float alpha, gamma;
    unsigned numActions;
    unsigned lastIndex;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> uniformDist;
    std::bernoulli_distribution bernoulliDist;
    // There may be an infinity number of states, thus states must be stored in dynamic vectors
    StatesContainer qlearnerStates;

    const unsigned fetchState( const std::vector<float> &observed_state );
};

#endif
