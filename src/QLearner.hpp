#ifndef QLearner_HPP_
#define QLearner_HPP_

#include <vector>
#include <algorithm>


struct State{
    State( unsigned num_actions, const std::vector<float> &observed_state );
    ~State();

    unsigned action;
    float maxQValue;
    std::vector<float>::iterator action_it;
    std::vector<float> stateValues;
    std::vector<float> QValues;

    bool compareState( const std::vector<float> &observed_state );
};

class QLearner{
    public:
    QLearner();
    ~QLearner();

    typedef std::vector<State> StatesContainer;
    const unsigned chooseAction( const std::vector<float> &observed_state );
    void updateQValues( const float& reward, const std::vector<float> &observed_state );

    private:
    float alpha, gamma;
    unsigned numActions;
    // There may be an infinity number of states, thus states must be stored in dynamic vectors
    StatesContainer qlearnerStates;
    StatesContainer::iterator lastState;
    StatesContainer::iterator fetchState( const std::vector<float> &observed_state );
};

#endif
