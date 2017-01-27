#!/usr/bin/env sh
compute_image_mean ./caffe/dataset/train_lmdb ./caffe/dataset/drl_gazebo_mean.binaryproto

echo "Done."
