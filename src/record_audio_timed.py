#!/usr/bin/env python

import rospy
import subprocess
import signal
import sys


child = subprocess.Popen(["roslaunch","poll_cameras","audio.launch"])

#rospy.loginfo('num arg: %d', len(sys.argv))
#rospy.loginfo('wait time: %f', float(sys.argv[1]))

rospy.sleep(float(sys.argv[1]))

child.send_signal(signal.SIGINT) #You may also use .terminate() method
#child.terminate()
