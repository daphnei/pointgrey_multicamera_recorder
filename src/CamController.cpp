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

#include <poll_cameras/CamController.h>
#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>

namespace poll_cameras {

  CamController::CamController(const ros::NodeHandle& parentNode, int numCameras)
    : parentNode_(parentNode),
      configServer_(parentNode),
      triggerThreads_(numCameras) {

    numCameras_ = numCameras;

    for (int i = 0; i<numCameras_; i++) {
      CamPtr cam_tmp = 
         boost::make_shared<Cam>(parentNode_, "cam" + std::to_string(i));

      cameras_.push_back(move(cam_tmp));
    }
    camera_ = cameras_[1];

    // for publishing the currently set exposure values
    expPub_ = parentNode_.advertise<std_msgs::Float64MultiArray>("current_exposure", 1);
    // camera_ = parentNode_.advertise<sensor_msgs::Image>("cam0", 1);

    expSub_ = parentNode_.subscribe("exposure", 1,
                                    &CamController::expCallback, this);
    configServer_.setCallback(boost::bind(&CamController::configure, this, _1, _2));

    // record_index = 0;
  }

  CamController::~CamController()
  {
    stopPoll();
  }
  
  void CamController::expCallback(const std_msgs::Float64MultiArray::ConstPtr &expMsg) {
    std::lock_guard<std::mutex> lock(expMutex_);
    double shutter(expMsg->data[0]), gain(expMsg->data[1]);
    if (shutter >= 0) {
      optimalShutter_ = shutter;
      updateShutter_  = true;
    }
    if (gain >= 0) {
      optimalGain_ = gain;
      updateGain_ = true;
    }
  }

  // void CamController::startRecording() {
    // char bag_name[15];
    // strftime(&bag_namr, 15, "%y_%m_%d")
    // sprintf(&bag_name, "%s%03d.bag", bag_name, record_index);
    // recorder.open(bag_name);
// 
    // recorder.record("/cam_test/cam0/image_raw", msg, ros::Time::now());
  // }

  void CamController::setShutter(double s) {
    bool auto_shutter(false);
    double rs(s);
    for (int i=0; i<numCameras_; ++i) {
       cameras_[i]->camera().SetShutter(auto_shutter, rs);
    }
    ROS_INFO("set shutter to: %8.4fms, driver returned %8.4fms", s, rs);}

  void CamController::setGain(double g) {
    bool auto_gain(false);
    double rg(g);
    for (int i=0; i<numCameras_; ++i) cameras_[i]->camera().SetGain(auto_gain, rg);
    ROS_INFO("set gain to:    %8.4fms, driver returned %8.4fms", g, rg);
  }

  void CamController::configure(Config& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    stopPoll();
    stopSoftTrigger();
    triggerSleepTime_ = 1000000 / config.fps;
    configureCameras(config);
    startPoll();
    startSoftTrigger();
  }

  void CamController::pollImages() {

    // first get the current camera settings and
    // publish them for the exposure control module
    optimalShutter_ = cameras_[0]->camera().GetShutterTimeSec() * 1000;
   
    // TODO: figure out why doesn't work 
    // optimalGain_    = camera_->camera().GetGain();
    optimalGain_    = 0; 

    publishCurrentExposure();
    lastPublishTime_ = ros::Time::now();
    ROS_INFO("node up and running!");
    while (isPolling_ && ros::ok()) {
      ros::Time time = ros::Time::now();

      for (int i=0; i<numCameras_; ++i) {
        CamPtr curCam = cameras_[i];
        auto image_msg = boost::make_shared<sensor_msgs::Image>();
        bool ret = curCam->Grab(image_msg);
        image_msg->header.stamp = time;
        // Publish takes less then 0.1ms to finish, so it is safe to put it here
        // in the loop
        if (ret) curCam->Publish(image_msg);
      }
      updateExposure();

      if (time - lastPublishTime_ > publishExposureInterval_) {
        publishCurrentExposure();
        lastPublishTime_ = time;
      }
    }
  }


  void CamController::updateExposure() {
    std::lock_guard<std::mutex> lock(expMutex_);
    if (updateShutter_) {
      setShutter(optimalShutter_);
      updateShutter_ = false;
    }
    if (updateGain_) {
      setGain(optimalGain_);
      updateGain_ = false;
    }
  }

  void CamController::publishCurrentExposure() {
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(optimalShutter_);
    msg.data.push_back(optimalGain_);
    expPub_.publish(msg);
  }

  void CamController::configureCameras(Config& config) {
    for (int i=0; i<numCameras_; ++i) {
      CamPtr curCam = cameras_[i];
      curCam->Stop();
      curCam->camera().Configure(config);
      curCam->set_fps(config.fps);
      curCam->Start();
    }
  }

  void CamController::startPoll() {
    isPolling_ = true;
    for (int i=0; i<numCameras_; ++i) {
      imgPollThread_ =
          boost::make_shared<boost::thread>(&CamController::pollImages, this);
    }
  }

  void CamController::stopPoll() {
    if (!isPolling_) return;
    isPolling_ = false;
    imgPollThread_->join();
  }

  void CamController::startSoftTrigger() {
    isTriggering_ = true;
    for (int i=0; i<numCameras_; ++i) {
      triggerThreads_[i] =
      boost::make_shared<boost::thread>(&CamController::triggerThread, this, i);
    }
  }

  void CamController::stopSoftTrigger() {
    if (!isTriggering_) return;
    isTriggering_ = false;
    for (int i=0; i<numCameras_; ++i) {
      triggerThreads_[i]->join();
    }
  }

  void CamController::triggerThread(int index) {
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);
      cameras_[index]->camera().FireSoftwareTrigger();
    }
  }

}  // namespace poll_cameras
