#!/bin/bash
pathDataset='/kitti/sequences/00/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Kitti RGB-L Mode"
./Examples/RGB-L/rgbl_kitti Vocabulary/ORBvoc.txt Examples/RGB-LExamples/RGB-D/KITTI04-12.yaml/KITTI00-02.yaml "$pathDataset"

