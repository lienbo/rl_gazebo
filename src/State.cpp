#include <algorithm>
#include "State.hpp"


using namespace std;

// Constructor used when loading policy
State::State( vector<float> qvalues, const vector<float> &state ) : QValue(0),
        maxQValue(0), action(0), convergedState( qvalues.size(), true ),
        convergenceTreshold( 0.6 )
{
    QValues = qvalues;
    stateValues = state;
}


State::State( unsigned num_actions, const vector<float> &observed_state ) : QValue(0),
        maxQValue(0), action(0), convergedState( num_actions, false ),
        convergenceTreshold( 0.6 )
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
