Readme
------

This package contains Gazebo plugins intended to simulate reinforcement learning algorithms in mobile robots.


Installation
------------

Supported system: Ubuntu 16.04 Xenial
Basic dependencies: cmake, gcc, opencv

1. Install Gazebo 7
    sudo apt-get install libgazebo7-dev

2. Compile and install caffe on the system
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local

3. Compile and install the source code
    mkdir build && cd build
    cmake .. -DCOMPILE_DRL_PLUGIN=1
    make -j4 && sudo make install


Testing
-------

1. Edit the gazebo world file in gazebo/worlds/
2. Set the mode tag to test
3. select number of iterations in max_steps
4. Launch gazebo
    gazebo gazebo/worlds/rover.world --verbose
    gazebo gazebo/worlds/nfq_rover.world --verbose
    gazebo gazebo/worlds/drl_rover.world --verbose


Training
--------

1. Edit the gazebo world files (rover.world, nfq_rover.world, drl_rover.world):
    set the mode tag to train
    select number of iterations in max_steps

2. Launch gazebo rover world:
    gazebo gazebo/worlds/rover.world --verbose

3. Convert the q-learning dataset:
    python caffe/scripts/convert_policy.py

4. Train the nfq network:
    bash caffe/scripts/train.sh nfq

5. Launch gazebo nfq rover world:
    gazebo gazebo/worlds/nfq_rover.world --verbose

6. Convert the nfq dataset:
    python caffe/scripts/convert_policy.py

7. Train the drl network:
    bash caffe/scripts/train.sh drl

8. Launch gazebo drl rover world:
    gazebo gazebo/worlds/drl_rover.world --verbose


Acknowledgement
---------------

This work is part of Thomio Watanabe PhD project funded by grant: #2015/26293-0, São Paulo Research Foundation (FAPESP).
"Opinions, hypothesis and conclusions or recommendations expressed herein are the author(s) responsibility and do not necessarily conform with FAPESP vision."  

Copyright © 2017 Thomio Watanabe
