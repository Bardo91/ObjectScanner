////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <rgbd_slam/utils/Gui.h>
#include <MainApplication.h>

#include <thread>
#include <chrono>

using namespace pcl;
int main(int _argc, char ** _argv) {

	// Create application
	app::fruitSearcher::MainApplication app;
	
	// Initialize with input parameters
	if (!app.init(_argc, _argv)) {
		return -1;
	}
	// Iterate.
	int stepCounter = 0;
	bool result = false;
	do {
		std::cout << "-------------------------------- Step: " << stepCounter++ <<"-------------------------------------" << std::endl;
		result = app.step();
	}while(/*result &&*/ !rgbd::Gui::get()->hasReceivedStopSignal());
	
	return 0;
}
