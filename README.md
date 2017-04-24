Readme
------

This package contains Gazebo plugins intended to simulate autonomous vehicles.

Test the simulation:
1 - Edit the gazebo world file:
    set the mode tag to test
    select number of iterations in max_steps

2 - Launch gazebo
    gazebo gazebo/worlds/rover.world --verbose
    gazebo gazebo/worlds/nfq_rover.world --verbose
    gazebo gazebo/worlds/drl_rover.world --verbose


Train the simulation:
1 - Edit the gazebo world files (rover.world, nfq_rover.world, drl_rover.world):
    set the mode tag to train
    select number of iterations in max_steps

2 - Launch gazebo rover world
    gazebo gazebo/worlds/rover.world --verbose

3 - Convert the q-learning dataset:
    python caffe/scripts/convert_policy.py

4 - Train the nfq network:
    bash caffe/scripts/train.sh nfq

5 - Launch gazebo nfq rover world:
    gazebo gazebo/worlds/nfq_rover.world --verbose

6 - Convert the nfq dataset:
    python caffe/scripts/convert_policy.py

7 - Train the drl network:
    bash caffe/scripts/train.sh drl

8 - Launch gazebo drl rover world:
    gazebo gazebo/worlds/drl_rover.world --verbose


Acknowledgement
---------------

This work is part of Thomio Watanabe PhD project funded by grant: #2015/26293-0, São Paulo Research Foundation (FAPESP).  
"Opinions, hypothesis and conclusions or recommendations expressed herein are the author(s) responsibility and do not necessarily conform with FAPESP vision."  

Copyright © 2017 Thomio Watanabe
