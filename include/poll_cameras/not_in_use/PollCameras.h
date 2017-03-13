#ifndef POLL_CAMERAS_H
#define POLL_CAMERAS_H

#include "poll_cameras/CamController.h"
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace poll_cameras {
class PollCameras {

public:
	PollCameras();
	// ~PollCameras();

	void startPoll( poll_cameras::CamController& cam );
	void stopPoll();

	bool isPolling_;
	boost::shared_ptr<boost::thread>      imgPollThread_;

private:
	void pollImages( poll_cameras::CamController& cam );

};

}
#endif
