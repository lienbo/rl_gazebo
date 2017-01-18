#!/usr/bin/env sh
set -e

$CAFFE_ROOT/build/tools/caffe train --solver=./caffe/network/lenet_solver.prototxt $@
