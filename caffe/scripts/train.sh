#!/usr/bin/env sh
set -e

$CAFFE_ROOT/build/tools/caffe train --solver=./caffe/network/drl_gazebo_solver.prototxt $@
