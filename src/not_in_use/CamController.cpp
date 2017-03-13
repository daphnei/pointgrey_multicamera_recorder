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

  CamController::CamController(const ros::NodeHandle& parentNode, int index)
    : parentNode_(parentNode), configServer_(parentNode) {

    sprintf(name_, "cam%d", index); 

    camera_ = boost::make_shared<Cam>(parentNode_, name_);
    camera_->set_fps(150.0);
    // for publishing the currently set exposure values

    char expName[40];
    sprintf(expName, "%s/current_exposure", name_);

    expPub_ = parentNode_.advertise<std_msgs::Float64MultiArray>(expName, 1);

    // This was commented out.
    // camera_ = parentNode_.advertise<sensor_msgs::Image>(name_, 1);

    expSub_ = parentNode_.subscribe("exposure", 1,
                                    &CamController::expCallback, this);
    configServer_.setCallback(boost::bind(&CamController::configure, this, _1, _2));
  }

  CamController::~CamController()
  {
    // stopPoll();
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
    camera_->camera().SetShutter(auto_shutter, rs);
    ROS_INFO("set shutter to: %8.4fms, driver returned %8.4fms", s, rs);}

  void CamController::setGain(double g) {
    bool auto_gain(false);
    double rg(g);
    camera_->camera().SetGain(auto_gain, rg);
    ROS_INFO("set gain to:    %8.4fms, driver returned %8.4fms", g, rg);
  }

  void CamController::configure(Config& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    // stopPoll();
    stopSoftTrigger();
    triggerSleepTime_ = 1000000 / config.fps;
    configureCameras(config);
    // startPoll();
    startSoftTrigger();
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
    camera_->Stop();
    camera_->camera().Configure(config);
    camera_->set_fps(config.fps);
    camera_->Start();
  }
  void CamController::startSoftTrigger() {
    isTriggering_ = true;
    triggerThread_ =
      boost::make_shared<boost::thread>(&CamController::triggerThread, this);
  }

  void CamController::stopSoftTrigger() {
    if (!isTriggering_) return;
    isTriggering_ = false;
    triggerThread_->join();
  }

  void CamController::triggerThread() {
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);
      camera_->camera().FireSoftwareTrigger();
    }
  }

}  // namespace poll_cameras
