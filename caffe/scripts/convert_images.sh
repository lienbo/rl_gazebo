#!/bin/bash
convert_imageset ./gazebo/output/images/ ./caffe/dataset/train.txt ./caffe/dataset/train_lmdb

echo "Done."
