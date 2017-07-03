#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "QLearner.hpp"


using namespace std;

// alpha = learning rate
// gama = discount factor
QLearner::QLearner( const unsigned &num_actions ) : alpha(0.4), gamma(0.8),
        numActions( num_actions ), uniformDist(0,num_actions - 1),
        outputDir("./gazebo/output/policy/")
{
    qlearnerStates.clear();
    generator.seed(time(0));
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


const unsigned QLearner::fetchState( const vector<float> &observed_state, vector<float> qvalues )
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
        state_it->QValues = qvalues;
    }

    // The action is the iterator position.
    // Never change the vector order (sort)
    vector<float>::iterator action_it = max_element( state_it->QValues.begin(), state_it->QValues.end() );
    state_it->action = distance( state_it->QValues.begin(), action_it );
    state_it->maxQValue = *action_it;
    state_it->QValue = *action_it;

    const unsigned state_index = distance( qlearnerStates.begin(), state_it );
    return state_index;
}


const unsigned QLearner::chooseAction( const unsigned &state_index, const float &probability )
{
    State &current_state = qlearnerStates[state_index];

    bernoulli_distribution bernoulli( probability );
    // Bernoulli distribution to change action
    if( bernoulli(generator) ){
        // Equal (uniform) probability to choose an action
        const unsigned action = uniformDist(generator);
        current_state.action = action;
        current_state.QValue = current_state.QValues[action];
    }

    lastIndex = state_index;

    return current_state.action;
}


const unsigned QLearner::selectAction( const unsigned &state_index )
{
    State &current_state = qlearnerStates[state_index];
    lastIndex = state_index;

    printQValues( "QValues = ", state_index );

    return current_state.action;
}


// Updated a state when an action is selected from an external agent
void QLearner::updateAction( const unsigned &state_index, unsigned action )
{
    State &current_state = qlearnerStates[state_index];
    current_state.action = action;
    current_state.QValue = current_state.QValues[action];

    lastIndex = state_index;
}


// Reward is related to the transition from last state to the current state
// Only the last state is updated
void QLearner::updateQValues( const float& reward, const unsigned &state_index )
{
    printQValues( "Old QValues = ", lastIndex );


    const unsigned current_index = state_index;
    State &last_state = qlearnerStates[lastIndex];

    // This is a modified version of the usual QLearning algorithms.
    // This updates the last QValues state instead of the current one.
    // lastAction points to the qvalue associated with the last action.
//    cout << "qmax = " << qlearnerStates[current_index].maxQValue << endl;

    float learned_value = reward + gamma * (qlearnerStates[current_index].maxQValue);
//    if( last_state.convergenceTreshold > last_state.QValue/learned_value )
    last_state.convergedState[ last_state.action ] = true;

    float qvalue_increment = alpha * ( learned_value - last_state.QValue );
    last_state.QValues[ last_state.action ] += qvalue_increment;

    printQValues( "New QValues = ", lastIndex );
}


void QLearner::updateQValues( const float& reward )
{
    State &last_state = qlearnerStates[lastIndex];

    // This is a modified version of the usual QLearning algorithms.
    // This updates the last QValues state instead of the current one.
    // lastAction points to the qvalue associated with the last action.
    last_state.QValues[ last_state.action ] += alpha * ( reward + gamma * ( 0 ) - (last_state.QValue) );
}


void QLearner::printQValues( const string &message, const unsigned &current_index ) const
{
    vector<float> qvalues = qlearnerStates[ current_index ].QValues;
    cout << message;
    for(vector<float>::const_iterator it = qvalues.begin();
            it != qvalues.end(); ++it ){
        cout << " " << *it;
    }
    cout << endl;

    vector<float>::iterator action_it = max_element( qvalues.begin(), qvalues.end() );
    cout << "Max qvalues = " << *action_it << endl;
    cout << "action = " <<  distance( qvalues.begin(), action_it ) << endl;
}


void QLearner::savePolicy( bool save_converged, bool standardize )
{
    // Save a NEW files with qvalues and state.
    // Old files are lost
    boost::filesystem::path dir( outputDir.c_str() );
    boost::filesystem::create_directories( dir );

    string state_file_name = outputDir + "qlearner_states.txt";
    string policy_file_name = outputDir + "qlearner_policy.txt";

    if( standardize ){
        float mean = 0;
        unsigned counter = 0;
        for(StatesContainer::iterator it = qlearnerStates.begin();
                it != qlearnerStates.end(); ++it){
            mean = accumulate( it->QValues.begin(), it->QValues.end(), mean );
            counter += it->QValues.size();
        }
        mean /= counter;

        float sum_squared_difference = 0;
        for(StatesContainer::iterator it = qlearnerStates.begin();
                it != qlearnerStates.end(); ++it){
            for(vector<float>::iterator qvalue_it = it->QValues.begin();
                    qvalue_it != it->QValues.end(); ++qvalue_it){
                sum_squared_difference += pow(*qvalue_it - mean, 2);
            }
        }
        float standard_deviation = sqrt( sum_squared_difference / counter );

        // Update qvalues
        for(StatesContainer::iterator it = qlearnerStates.begin();
                it != qlearnerStates.end(); ++it){
            for(vector<float>::iterator qvalue_it = it->QValues.begin();
                    qvalue_it != it->QValues.end(); ++qvalue_it){
                *qvalue_it = (*qvalue_it - mean)/standard_deviation;
            }
        }
    }

    ofstream policy_file, state_file;
    policy_file.open( policy_file_name.c_str(), ios::out );
    state_file.open( state_file_name.c_str(), ios::out );

    if( save_converged ){
        vector<bool> true_vector( numActions, true  );
        for(StatesContainer::iterator it = qlearnerStates.begin();
                it != qlearnerStates.end(); ++it){
            if( equal( it->convergedState.begin(), it->convergedState.end(), true_vector.begin() ) )
            {
                ostream_iterator<float> qvalue_iterator(policy_file, " ");
                copy(it->QValues.begin(), it->QValues.end(), qvalue_iterator);
                policy_file << "\n";

                ostream_iterator<float> state_iterator(state_file, " ");
                copy(it->stateValues.begin(), it->stateValues.end(), state_iterator);
                state_file << "\n";
            }
        }
    }else {
        for(StatesContainer::iterator it = qlearnerStates.begin();
                it != qlearnerStates.end(); ++it){
            ostream_iterator<float> qvalue_iterator(policy_file, " ");
            copy(it->QValues.begin(), it->QValues.end(), qvalue_iterator);
            policy_file << "\n";

            ostream_iterator<float> state_iterator(state_file, " ");
            copy(it->stateValues.begin(), it->stateValues.end(), state_iterator);
            state_file << "\n";
        }
    }

    policy_file.close();
    state_file.close();
}


void QLearner::loadPolicy()
{
    qlearnerStates.clear();

    string state_file_name = outputDir + "qlearner_states.txt";
    string policy_file_name = outputDir + "qlearner_policy.txt";
    ifstream policy_file, state_file;
    policy_file.open( policy_file_name.c_str(), ios::in );
    state_file.open( state_file_name.c_str(), ios::in );

    string policy_line, state_line;
    while( getline( policy_file, policy_line) && getline( state_file, state_line ) ){
        vector<float> qvalues, state_values;
        vector<string> qvalues_str, state_str;

        boost::split( qvalues_str, policy_line, boost::is_any_of(" ") );
        // Remove /n char
        qvalues_str.pop_back();
        for(vector<string>::iterator it = qvalues_str.begin(); it != qvalues_str.end(); ++it)
            qvalues.push_back( stof(*it) );

        boost::split( state_str, state_line, boost::is_any_of(" ") );
        state_str.pop_back();
        for(vector<string>::iterator it = state_str.begin(); it != state_str.end(); ++it)
            state_values.push_back( stof(*it) );

        State state( qvalues, state_values );
        qlearnerStates.push_back( state );
    }

    policy_file.close();
    state_file.close();
}
