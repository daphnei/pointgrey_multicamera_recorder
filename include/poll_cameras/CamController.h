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

#ifndef EXPOSURECONTROL_CAMTEST_H
#define EXPOSURECONTROL_CAMTEST_H

#include <mutex>
#include <boost/thread.hpp>
#include <condition_variable>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <time.h>
#include <ros/message_traits.h>
#include <dynamic_reconfigure/server.h>
#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>

namespace flea3 {
  class Flea3Ros;
}

namespace poll_cameras {

class CamController {
  
public:
  using Cam = flea3::Flea3Ros;
  using CamPtr = boost::shared_ptr<Cam>;
  using ThreadPtr = boost::shared_ptr<boost::thread>;
  using Config = flea3::Flea3DynConfig;
  using Time = ros::Time;

  CamController(const ros::NodeHandle& parentNode, int numCameras);
  ~CamController();
  CamController(const CamController&) = delete;
  CamController& operator=(const CamController&) = delete;

  void configure(Config& config, int level);
  void expCallback(const std_msgs::Float64MultiArray::ConstPtr &expMsg);

  void setShutter(double s);
  void setGain(double g);
  void publishCurrentExposure();
  void updateExposure();
  void startPoll();
  void stopPoll();
  void startSoftTrigger();
  void stopSoftTrigger();
  void configureCameras(Config& config);

private:
  // Thread functions are private.
  void pollImages();
  void triggerThread();
  void frameGrabThread(int camIndex);

  // Variables for the camera state
  ros::NodeHandle          parentNode_;
  ros::Subscriber          expSub_;
  ros::Publisher           expPub_;
  std::mutex               expMutex_;
  std::mutex               grabFramesMutex_;
  std::condition_variable  grabFramesCV_;
  int                      numFramesToGrab_;
  int                      numCameras_;
  Time                frameTime_;
  std::vector<CamPtr>      cameras_;
  bool                     isPolling_{false};
  Time                lastPublishTime_;
  ros::Duration            publishExposureInterval_{1.0};  // in seconds
  double                   optimalShutter_{10};
  double                   optimalGain_{0};
  bool                     updateShutter_{false};
  bool                     updateGain_{false};
  int                      triggerSleepTime_{100000}; // in usec
  bool                     isTriggering_{false};
  ThreadPtr                imgPollThread_;
  ThreadPtr                triggerThread_;
  std::vector<ThreadPtr>   frameGrabThreads_;
  dynamic_reconfigure::Server<Config>   configServer_;

  long index_;
};

}

#endif
