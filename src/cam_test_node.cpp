/*
 * 2016 Bernd Pfrommer
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "poll_cameras/CamTest.h"
#include <ros/ros.h>



int main(int argc, char** argv) {
  ROS_INFO("Initializaing Camera Polling Node");

  ros::init(argc, argv, "cam_test");
  ros::NodeHandle nh("~");

  try {
    poll_cameras::CamTest cam(nh);

    while (1) {
      char c = getchar();
      switch (c) {
        case '1':
          cam.startPoll();
          ROS_INFO("Starting polling camera.");
          break;
        case '2':
          cam.stopPoll();
          ROS_INFO("Stopping polling camera.");
          break;
        case 'q':
          cam.stopPoll();
          ROS_INFO("Exiting");
          exit(1);
        default:
          ROS_INFO("Unrecognized input.");
      }
      // Gobble up the rest, just cared about the first char.
      while((getchar())!='\n');
    }

    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
