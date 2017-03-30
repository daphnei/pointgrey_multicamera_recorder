#!/usr/bin/python

import rosbag
import rospy
import sys
import pdb
import os
import glob
import numpy as np

if __name__ == "__main__":
	rospy.init_node('bag_analyzer')
	args = rospy.myargv(argv=sys.argv)

	bags_path = rospy.get_param("~bags_path", "/data")
	num_cameras = int(rospy.get_param("~num_cameras", 1))

	t_dict = {}
	for bag_file in glob.glob(os.path.join(bags_path, "*.bag")):
		bag = rosbag.Bag(bag_file)
		print("Bag: " + bag_file)
		print("Num messages: " + str(bag.get_message_count()))
		print("Start time: "   + str(bag.get_start_time()))
		print("End time: "     + str(bag.get_end_time()))
		print("First 20 times: ")
		i = 0
		time_diffs = []
		last_timestamp = None
   		for topic, msg, _ in bag.read_messages(topics=[]):
			timestamp = (10e9 * msg.header.stamp.secs) + msg.header.stamp.nsecs
 
			if i >= 1:
				time_diffs.append(timestamp - last_timestamp)
			last_timestamp = timestamp

			if (i < 20):
				print timestamp
				i = i+1

			# t = t.to_sec()
			if timestamp in t_dict:
				t_dict[timestamp] += 1
			else:
				t_dict[timestamp] = 1
	
		time_diffs = np.array(time_diffs)
		mean_diff = np.mean(time_diffs)
		std_diff = np.std(time_diffs)
		print("Mean time diff (seconds): " + str(10e-9 * mean_diff)) # + " ( " + str(1/(mean_diff* 1.0e-9)) + " FPS )")
		print("FPS: " + str(1 / (10e-9 * mean_diff)))
		print("Stdv time diff (seconds): " + str(10e-9 * std_diff))

		print("\n")
		bag.close()

	for i in xrange(1, num_cameras + 1):
		num_times = sum(1 for x in t_dict.values() if x == i)
		print(str(num_times) + " timestamps occur " + str(i) + " times.")

