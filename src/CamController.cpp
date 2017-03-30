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

#define THREAD_GUARD 0

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

	
	// Retrieve all of the camera parameters from the ros node
	bool autoShutter;
	parentNode.getParam("auto_shutter", autoShutter);
	double fps;
	parentNode.getParam("fps", fps);
	double shutterMs;
	parentNode.getParam("shutter_ms", shutterMs);
   
	ROS_INFO_STREAM("Desired frame rate: " << fps); 
	// Ensure software trigger is turned on.
	int trigger(flea3::Flea3Dyn_ts_sw);
    int polarity(0);

    for (int i = 0; i<numCameras_; i++) {
      CamPtr cam_tmp = 
         boost::make_shared<Cam>(parentNode_, "cam" + std::to_string(i));

      cameras_.push_back(move(cam_tmp));

      // Blatant hard-coding to see if it solves anything.
      cameras_[i]->camera().SetShutter(autoShutter, shutterMs);
      cameras_[i]->camera().SetFrameRate(fps);
      cameras_[i]->camera().SetTrigger(trigger, polarity);
    }

	triggerSleepTime_ = 1e6 / fps;

    // for publishing the currently set exposure values
    expPub_ = parentNode_.advertise<std_msgs::Float64MultiArray>("current_exposure", 1);

    configServer_.setCallback(boost::bind(&CamController::configure, this, _1, _2));
  }

  CamController::~CamController()
  {
    stopPoll();
  }
  
  void CamController::configure(Config& config, int level) {
#if 0
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
#endif    
  }

  void CamController::frameGrabThread(int camIndex) {
	ROS_INFO("Starting up frame grab thread for camera %d", camIndex);

    while (isPolling_ && ros::ok()) {

#if THREAD_GUARD
      std::unique_lock<std::mutex> lock(grabFramesMutex_);

      while((numFramesToGrab_ & (1 << camIndex)) == 0) {
        grabFramesCV_.wait(lock);
      } 
#endif

      // Grab a copy of the time, then can release the lock so that 
      // trigger can start a new frame if necessary.
      Time myFrameTime = Time(frameTime_);

#if THREAD_GUARD
      numFramesToGrab_ = (numFramesToGrab_ ^ (1 << camIndex));
      lock.unlock();
      grabFramesCV_.notify_all(); 
      // ROS_INFO_STREAM("Grabbing frame at time " << myFrameTime);
#endif

      CamPtr curCam = cameras_[camIndex];
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      bool ret = curCam->Grab(image_msg);
      image_msg->header.stamp = myFrameTime;
      // Publish takes less then 0.1ms to finish, so it is safe to put it        
      // in the loop
      // ROS_INFO("g: Grabbing from %d at time %f", camIndex, myFrameTime.toSec());
      if (ret) {
        curCam->Publish(image_msg);
      } else {
        ROS_ERROR("There was a problem grabbing a frame from cam%d", camIndex);
      }
    }
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
      cameras_[i]->camera().StartCapture();
      frameGrabThreads_[i] =
      	boost::make_shared<boost::thread>(&CamController::frameGrabThread, this, i);
    }

  }

  void CamController::stopPoll() {
    if (!isPolling_) return;

    isPolling_ = false;
    for (int i=0; i<numCameras_; ++i) {
      frameGrabThreads_[i]->join();
    }

    for (int i=0; i<numCameras_; ++i) {
      cameras_[i]->camera().StopCapture();
    }

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
    ros::Time t0 = ros::Time::now();
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);

#if THREAD_GUARD
      std::unique_lock<std::mutex> lock(grabFramesMutex_);
      while (isPolling_ && numFramesToGrab_ > 0) {
        grabFramesCV_.wait(lock);
      }
#endif

      frameTime_ = ros::Time::now();
      ROS_INFO_STREAM("time for sleep: " << (frameTime_ - t0) << ", should be " << (triggerSleepTime_ / 1e6));
      
      for (int i=0; i<numCameras_; i++) {
        cameras_[i]->camera().FireSoftwareTrigger();
      }
      ros::Time t1 = ros::Time::now();
      ROS_INFO_STREAM("time for triggering: " << (t1 - frameTime_));
      t0 = t1;

#if THREAD_GUARD
      numFramesToGrab_ = int(pow(2, numCameras_)) - 1;
      lock.unlock();
      grabFramesCV_.notify_all(); 
#endif
    }
  }

}  // namespace poll_cameras
