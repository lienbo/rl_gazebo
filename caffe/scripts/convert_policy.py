#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import h5py
import os


policy_file = open('./output/policy/qlearner_policy.txt')

policy = []
for line in policy_file:
    policy_list = line.split()
    policy.append( policy_list )

policy_file.close()

policy_array = np.asarray( policy, dtype=np.float32 )
print( 'Policy shape = ', policy_array.shape )
num_states = policy_array.shape[0]
num_actions = policy_array.shape[1]

print( 'Generating policy h5 file...')
print( 'Number of states = ', num_states )
print( 'Number of actions = ', num_actions )


h5_filename = './caffe/dataset/train_policy.h5'
with h5py.File( h5_filename, 'w' ) as h5_file:
    h5_file['policy'] = policy_array

with open('./caffe/dataset/train_policy.txt', 'w') as h5_txt:
    h5_txt.write( h5_filename )




states_file = open('./output/policy/qlearner_states.txt')

states = []
for line in states_file:
    states_list = line.split()
    states.append( states_list )

states_file.close()

states_array = np.asarray( states, dtype=np.float32 )
print( 'States array shape = ', states_array.shape )
num_states = states_array.shape[0]
num_dimensions = states_array.shape[1]

print( 'Generating states h5 file...')
print( 'Number of states = ', num_states )
print( 'Number of dimensions = ', num_dimensions )


h5_filename = './caffe/dataset/train_states.h5'
with h5py.File( h5_filename, 'w' ) as h5_file:
    h5_file['states'] = states_array

with open('./caffe/dataset/train_states.txt', 'w') as h5_txt:
    h5_txt.write( h5_filename )
