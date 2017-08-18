#!/bin/bash

duration=10
if [ ! -z "$1" ]; then
   duration=$1
fi

echo "launching record!"
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd /data
roslaunch poll_cameras record.launch duration:=$duration
python ~/bag2video/bag2video.py /poll_cameras/cam0/image_raw,/poll_cameras/cam1/image_raw,/poll_cameras/cam2/image_raw,/poll_cameras/cam3/image_raw,/audio/audio rec.bag
dt=`date +'%Y-%m-%d-%H-%M-%S'`
mv rec.bag ${dt}.bag
mv rec.mp4 ${dt}.mp4
rm audio.mp3
ln -sf ${dt}.mp4 current.mp4
rm tmp_rec.mp4
