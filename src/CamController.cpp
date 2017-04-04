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
      frameGrabThreads_(numCameras) {
    ros::NodeHandle nh_cam("~/camconfig");
    ros::NodeHandle nh_contr("~/controllerconfig");
    camConfigServer_.reset(new dynamic_reconfigure::Server<CamConfig>(nh_cam));
    configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_contr));

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

    camConfigServer_->setCallback(boost::bind(&CamController::configureCams, this, _1, _2));
    configServer_->setCallback(boost::bind(&CamController::configure, this, _1, _2));
  }

  CamController::~CamController()
  {
    stopPoll();
  }

  void CamController::start() {
    time_ = ros::Time::now();  
    startPoll();
  }

  
  void CamController::configure(Config& config, int level) {
    ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
             "calling reconfigure server");

    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing reconfigure server");
    }
  }

  void CamController::configureCams(CamConfig& config, int level) {
    ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
             "calling camera reconfig");

    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing cam reconfigure server");
    }
    bool wasPolling = isPolling_;
    if (wasPolling) stopPoll();
    stopSoftTrigger();
    triggerSleepTime_ = 1000000 / config.fps;
    configureCameras(config);
    if (wasPolling) startPoll();
    startSoftTrigger();
  }

#define HACK
  void CamController::frameGrabThread(int camIndex) {
    ROS_INFO("Starting up frame grab thread for camera %d", camIndex);

    // Grab a copy of the time, then can release the lock so that 
    // trigger can start a new frame if necessary.
    ros::Time lastTime;
    {
      std::unique_lock<std::mutex> lock(timeMutex_);
      lastTime = time_;
    }
    Time t0; 
    while (isPolling_ && ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      CamPtr curCam = cameras_[camIndex];
      t0 = ros::Time::now();
      bool ret = curCam->Grab(image_msg);
      ros::Time t1 = ros::Time::now();  
      {  // this section is protected by mutex
        
        std::unique_lock<std::mutex> lock(timeMutex_);
        if (camIndex == masterCamIdx_) {
          // master camera updates the timestamp for everybody
          time_ = ros::Time::now();
          timeCV_.notify_all();
        } else {
          // slave cameras wait until the master has published
          // a new timestamp.
          while (time_ <= lastTime) {
            timeCV_.wait(lock); // lock will be free while waiting!
          }
          lastTime = time_;
        }
      }
      std::cout << camIndex << " " << time_ << " grab time: " << (t1-t0) << std::endl;
      if (ret) {
        curCam->Publish(image_msg);
      } else {
        ROS_ERROR("There was a problem grabbing a frame from cam%d", camIndex);
      }
    }
  }

  void CamController::configureCameras(CamConfig& config) {
    for (int i=0; i<numCameras_; ++i) {
      CamPtr curCam = cameras_[i];
      curCam->Stop();
      if (i == masterCamIdx_) {
        // Switch on strobe for master, and make it free running.
        // The master camera sets the pace.
        //config.enable_strobe2 = true;
        //config.strobe2_polarity = 0;
        //config.enable_trigger = false;
      } else {
        // The master camera sets the pace.
        //config.enable_trigger = true;
        //config.trigger_mode = 0; // simpl external trigger, see ptgrey docs
        //config.trigger_source = "gpio3";
        //config.trigger_polarity = 0;
      }
      curCam->camera().Configure(config);
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
#if 0    
    isTriggering_ = true;
    triggerThread_ =
      boost::make_shared<boost::thread>(
        &CamController::triggerThread, this);
#endif
  }

  void CamController::stopSoftTrigger() {
#if 0    
    if (!isTriggering_) return;
    isTriggering_ = false;
    triggerThread_->join();
#endif
  }

  void CamController::triggerThread() {
    ros::Time t0 = ros::Time::now();
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);

      frameTime_ = ros::Time::now();
      //ROS_INFO_STREAM("time for sleep: " << (frameTime_ - t0) << ", should be " << (triggerSleepTime_ / 1e6));
      
      for (int i=0; i<numCameras_; i++) {
        cameras_[i]->camera().FireSoftwareTrigger();
      }
      ros::Time t1 = ros::Time::now();
      //ROS_INFO_STREAM("time for triggering: " << (t1 - frameTime_));
      t0 = t1;
    }
  }

}  // namespace poll_cameras
