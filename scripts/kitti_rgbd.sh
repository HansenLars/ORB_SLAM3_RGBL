#!/bin/bash
pathDataset='/kitti/01/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Kitti RGB-D Mode"
./Examples/RGB-L/rgbl_kitti Vocabulary/ORBvoc.txt Examples/RGB-L/KITTI_AV21.yaml "$pathDataset"

