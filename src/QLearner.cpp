#include "QLearner.hpp"
#include <iostream>

using namespace std;


State::State( unsigned num_actions, const vector<float> &observed_state ) : QValue(0), maxQValue(0), action(0)
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



QLearner::QLearner( const unsigned &num_actions ) : alpha(0.1), gamma(0.95), numActions( num_actions ), uniformDist(0,num_actions - 1), bernoulliDist(0.5)
{
    qlearnerStates.clear();
}


QLearner::~QLearner() {}


const unsigned QLearner::fetchState( const vector<float> &observed_state )
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
    State &current_state = *state_it;
    vector<float> &qvalues = current_state.QValues;
    vector<float>::iterator action_it = max_element( qvalues.begin(), qvalues.end() );
    current_state.action = distance( qvalues.begin(), action_it );
    current_state.maxQValue = *action_it;
    current_state.QValue = *action_it;

    const unsigned state_index = distance( qlearnerStates.begin(), state_it );
    return state_index;
}



const unsigned QLearner::chooseAction( const vector<float> &observed_state, const bool &training )
{
    unsigned state_index = fetchState( observed_state );

    State &current_state = qlearnerStates[state_index];
    if( training ){
        // Bernoulli distribution to change action
        if( bernoulliDist(generator) ){
            // Equal (uniform) probability to choose an action
            const unsigned action = uniformDist(generator);
            current_state.action = action;
            current_state.QValue = current_state.QValues[action];
        }
    }

    lastIndex = state_index;

    return current_state.action;
}


// reward is related to the transition from last state to the current state
void QLearner::updateQValues( const float& reward, const vector<float> &observed_state )
{
    const unsigned current_index = fetchState( observed_state );
    State &last_state = qlearnerStates[lastIndex];

    // This is a modified version of the usual QLearning algorithms.
    // This updates the last QValues state instead of the current one.
    // lastAction points to the qvalue associated with the last action.
    last_state.QValues[ last_state.action ] += alpha * ( reward + gamma * (qlearnerStates[current_index].maxQValue) - (last_state.QValue) );
}
