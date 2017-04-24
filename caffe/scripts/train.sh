#!/usr/bin/env sh
set -e

if [ $# -ne 1 ]; then
  echo 'Wrong number of arguments'
  echo 'Usage: caffe/scripts/train.sh nfq'
  echo 'Usage: caffe/scripts/train.sh drl'
  exit 0
fi

if [ $1 == 'nfq' ]
  then
    caffe train --solver=./caffe/network/nfq_gazebo_solver.prototxt
fi

if [ $1 == 'drl' ]
  then
    caffe train --solver=./caffe/network/drl_gazebo_solver.prototxt
fi
