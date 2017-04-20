#!/bin/sh

duration=$1

# Save bag files with date as name to prevent overwriting.
roslaunch poll_cameras record.launch out_file_prefix:=$(date "+%Y_%m_%d_%H_%M" ) duration:=$duration
 
