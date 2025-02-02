#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

xhost +local:root

XSOCK=/tmp/.X11-unix && \
docker run -it \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v /media/lars/SSD/Semesterarbeit/Data_IAC/Datasets_Kitti_Style/Monza_Tunnel/:/kitti/ \
 -v $SCRIPT_DIR/../:/ORBSLAM3/ \
 --network=host \
 --privileged \
 rgbl:latest

 xhost -local:root
