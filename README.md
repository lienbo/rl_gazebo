Readme
------

This package contains Gazebo plugins intended to simulate reinforcement learning algorithms in mobile robots.


Installation
------------

Supported system: Ubuntu 16.04 Xenial  
Basic dependencies: cmake, gcc, opencv

1. Install Gazebo 7
    ```bash
    sudo apt-get install libgazebo7-dev
    ```

2. Compile and install [caffe](http://caffe.berkeleyvision.org/installation.html) on the system
    ```bash
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    ```

3. Compile and install the source code
    ```bash
    mkdir build && cd build
    cmake .. -DCOMPILE_DRL_PLUGIN=1
    make -j4 && sudo make install
    ```


Testing
-------

1. Edit the gazebo world file in gazebo/worlds/
    * set the mode tag to test
    * select number of iterations in max_steps
2. Launch gazebo
    ```bash
    gazebo gazebo/worlds/rover.world --verbose
    gazebo gazebo/worlds/nfq_rover.world --verbose
    gazebo gazebo/worlds/drl_rover.world --verbose
    ```


Training
--------

1. Edit the gazebo world files (rover.world, nfq_rover.world, drl_rover.world):
    * set the mode tag to train
    * select number of iterations in max_steps

2. Launch gazebo rover world:
    ```bash
        gazebo gazebo/worlds/rover.world --verbose
    ```


Acknowledgement
---------------

This work is part of Thomio Watanabe PhD project funded by grant: #2015/26293-0, São Paulo Research Foundation (FAPESP).
"Opinions, hypothesis and conclusions or recommendations expressed herein are the author(s) responsibility and do not necessarily conform with FAPESP vision."  

Copyright © 2017 Thomio Watanabe
