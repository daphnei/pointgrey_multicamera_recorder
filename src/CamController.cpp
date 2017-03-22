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
#include <math.h>

namespace poll_cameras {

  CamController::CamController(const ros::NodeHandle& parentNode, int numCameras)
    : parentNode_(parentNode),
      configServer_(parentNode),
      frameGrabThreads_(numCameras) {

    numCameras_ = numCameras;
	numFramesToGrab_ = 0;

    ros::Time foo = ros::Time::now();
    frameTime_ = ros::Time::now();
	index_ = 0;

    for (int i = 0; i<numCameras_; i++) {
      CamPtr cam_tmp = 
         boost::make_shared<Cam>(parentNode_, "cam" + std::to_string(i));

      cameras_.push_back(move(cam_tmp));
    }

    // for publishing the currently set exposure values
    expPub_ = parentNode_.advertise<std_msgs::Float64MultiArray>("current_exposure", 1);

    expSub_ = parentNode_.subscribe("exposure", 1,
                                    &CamController::expCallback, this);
    configServer_.setCallback(boost::bind(&CamController::configure, this, _1, _2));
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
	bool wasPolling = isPolling_;
	if (wasPolling) stopPoll();
    stopSoftTrigger();
    triggerSleepTime_ = 1000000 / config.fps;
    configureCameras(config);
    if (wasPolling) startPoll();
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
    ROS_INFO("Polling has started!");

    while (isPolling_ && ros::ok()) {
      updateExposure();

      if (frameTime_ - lastPublishTime_ > publishExposureInterval_) {
        publishCurrentExposure();
        lastPublishTime_ = frameTime_;
      }
    }
  }

  void CamController::frameGrabThread(int camIndex) {
	ROS_INFO("Starting up frame grab thread for camera %d", camIndex);

    while (isPolling_ && ros::ok()) {
      std::unique_lock<std::mutex> lock(grabFramesMutex_);

      while((numFramesToGrab_ & (1 << camIndex)) == 0) {
        grabFramesCV_.wait(lock);
		// ROS_INFO("Woken up in camIndex %d", camIndex);
        // ROS_INFO("g_woken: Frames to grab: %d, %f", numFramesToGrab_,
				// pow(2, camIndex));
      } 

      // Grab a copy of the time, then can release the lock so that 
      // trigger can start a new frame if necessary.
      Time myFrameTime = Time(frameTime_);
      // ROS_INFO("Camera %d grabbing %f", camIndex, frameTime_.toSec());

      numFramesToGrab_ = (numFramesToGrab_ ^ (1 << camIndex));
      // ROS_INFO("g: Frames to grab: %d", numFramesToGrab_);
      lock.unlock();
      grabFramesCV_.notify_all(); 

      CamPtr curCam = cameras_[camIndex];
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      bool ret = curCam->Grab(image_msg);
      image_msg->header.stamp = myFrameTime;
      // Publish takes less then 0.1ms to finish, so it is safe to put it        
      // in the loop
      if (ret) curCam->Publish(image_msg);

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
      frameGrabThreads_[i] =
      	boost::make_shared<boost::thread>(&CamController::frameGrabThread, this, i);
    }

    imgPollThread_ =
      boost::make_shared<boost::thread>(&CamController::pollImages, this);
  }

  void CamController::stopPoll() {
    if (!isPolling_) return;
    isPolling_ = false;
    imgPollThread_->join();
  }

  void CamController::startSoftTrigger() {
    isTriggering_ = true;
    triggerThread_ =
        boost::make_shared<boost::thread>(
            &CamController::triggerThread, this);
  }

  void CamController::stopSoftTrigger() {
    if (!isTriggering_) return;
    isTriggering_ = false;
    triggerThread_->join();
  }

  void CamController::triggerThread() {
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);

      std::unique_lock<std::mutex> lock(grabFramesMutex_);
      while (isPolling_ && numFramesToGrab_ > 0) {
        grabFramesCV_.wait(lock);
      }

      frameTime_ = ros::Time(index_++); //ros::Time::now();

      for (int i=0; i<numCameras_; i++) {
        cameras_[i]->camera().FireSoftwareTrigger();
      }

      numFramesToGrab_ = int(pow(2, numCameras_)) - 1;
      lock.unlock();
      grabFramesCV_.notify_all(); 

    }
  }

}  // namespace poll_cameras
