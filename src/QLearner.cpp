#include "QLearner.hpp"
#include <iostream>

using namespace std;


State::State( unsigned num_actions, const vector<float> &observed_state ) : maxQValue(0), action(0)
{
    // Each state is defined by its state values
    stateValues = observed_state;
    // The number of actions is a finite number
    QValues.resize(num_actions,0);
};


State::~State() {}


bool State::compareState( const vector<float> &observed_state )
{
    bool equal_states = false;
    if( equal(stateValues.begin(), stateValues.end(), observed_state.begin() ) ){
        equal_states = true;
    }

    return equal_states;
}



QLearner::QLearner( const unsigned &num_actions ) : alpha(0.1), gamma(0.95), numActions( num_actions )
{
    qlearnerStates.clear();
}


QLearner::~QLearner() {}


QLearner::StatesContainer::iterator QLearner::fetchState( const vector<float> &observed_state )
{
    StatesContainer::iterator state_it;
    for(state_it = qlearnerStates.begin(); state_it != qlearnerStates.end(); ++state_it){
        if( state_it->compareState( observed_state ) ){
            // Found state. Break loop
            break;
	    }
    }

    // Iterator is equal to qlearnerStates.end() if no state was found
    if( state_it == qlearnerStates.end() ){
        State new_state( numActions, observed_state );
        state_it = qlearnerStates.insert( state_it, new_state );
    }

    // The action is the iterator position.
    // Never change the vector order (sort)
    vector<float> qvalues = state_it->QValues;
    vector<float>::iterator action_it = max_element( qvalues.begin(), qvalues.end() );

    state_it->action = distance( qvalues.begin(), action_it );
    state_it->action_it = action_it;
    state_it->maxQValue = *action_it;

    return state_it;
}


const unsigned QLearner::chooseAction( const vector<float> &observed_state )
{
    StatesContainer::iterator state_it = fetchState( observed_state );
    lastState = state_it;

    return state_it->action;
}


// reward is related to the transition from last state to the current state
void QLearner::updateQValues( const float& reward, const vector<float> &observed_state )
{
    StatesContainer::iterator current_state = fetchState( observed_state );

    // This is a modified version of the usual QLearning algorithms.
    // This updates the last QValues state instead of the current one.
    // lastAction points to the qvalue associated with the last action.
    *(lastState->action_it) += alpha * ( reward + gamma * (current_state->maxQValue) - (lastState->maxQValue) );
}
