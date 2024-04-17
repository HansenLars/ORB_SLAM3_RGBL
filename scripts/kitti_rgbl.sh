#!/bin/bash
pathDataset='/kitti/02/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Kitti RGB-L Mode"
./Examples/RGB-L/rgbl_kitti Vocabulary/ORBvoc.txt Examples/RGB-L/KITTIAV-21.yaml "$pathDataset"

