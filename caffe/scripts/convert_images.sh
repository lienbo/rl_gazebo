#!/bin/bash
$CAFFE_ROOT/build/tools/convert_imageset ./output/images/ ./caffe/dataset/train.txt ./caffe/dataset/train_lmdb

echo "Done."
