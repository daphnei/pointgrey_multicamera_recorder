#!/bin/bash

# duration of recording, in seconds
duration=15
if [ ! -z "$1" ]; then
   duration=$1
fi

# delay of audio recording, in seconds
audio_delay=0
if [ ! -z "$2" ]; then
   audio_delay=$2
fi

# now run the aplay in the background (ampersand at end of line!), with some delay
(sleep $audio_delay; aplay ~/Music/bdynoise.wav) &

# launch the record and processing
~/catkin_ws/src/poll_cameras/src/record_and_process.bash $duration
