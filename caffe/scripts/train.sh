#!/usr/bin/env sh
set -e

caffe train --solver=./caffe/network/drl_gazebo_solver.prototxt $@
