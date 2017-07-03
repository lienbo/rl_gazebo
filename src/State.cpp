#include <algorithm>
#include "State.hpp"


using namespace std;

// Constructor used when loading policy
State::State( vector<float> qvalues, const vector<float> &observed_state ) :
        QValue(0), maxQValue(0), action(0),
        convergedState( qvalues.size(), true ), convergenceTreshold( 0.6 )
{
    stateValues = observed_state;
    QValues = qvalues;
    updateMaxQValue();
}


State::State( unsigned num_actions, const vector<float> &observed_state ) :
        QValue(0), maxQValue(0), action(0),
        convergedState( num_actions, false ), convergenceTreshold( 0.6 )
{
    // Each state is defined by its state values
    stateValues = observed_state;
    // The number of actions is a finite number
    QValues.resize(num_actions, 0);
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


void State::updateMaxQValue()
{
    // Search for the highest qvalue and update maxQValue
    vector<float>::iterator action_it = max_element( QValues.begin(), QValues.end() );
    maxQValue = *action_it;
}


bool State::abs_compare(float a, float b)
{
    return (abs(a) < abs(b));
}


// Scaling the QValues is not a required step for QLearning
// Divide QValues by the max absolute value -> QValues E [-1,1]
void State::scaleQValues()
{
    vector<float>::iterator abs_value_it = max_element(QValues.begin(), QValues.end(), abs_compare);
    float max_value = abs( *abs_value_it );

    if( max_value != 0 ){
        for( vector<float>::iterator it = QValues.begin(); it != QValues.end(); ++it ){
            *it = *it / max_value;
        }
        updateMaxQValue();
    }
}
