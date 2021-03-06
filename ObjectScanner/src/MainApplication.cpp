////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "MainApplication.h"

#include <rgbd_slam/utils/Gui.h>

#include <fstream>
#include <iostream>
#include <chrono>

#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace pcl;
using namespace rgbd;
using namespace Eigen;

namespace app {
	namespace ObjectScanner {
		//---------------------------------------------------------------------------------------------------------------------
		bool MainApplication::init(int _argc, char ** _argv) {
			// Check minimal arguments
			if (_argc != 2) {
				std::cout << "[MAIN APPLICATION] Wrong input parameters. Provide a path to a JSON file." << std::endl;
				return false;
			}
			std::ifstream file(_argv[1]);
			if (!file.is_open()) {
				std::cout << "[MAIN APPLICATION] Can't open config file" << std::endl;
				return false;
			}

			if (!mConfigFile.parse(file)) {
				std::cout << "[MAIN APPLICATION] Can't parse properly JSON config file" << std::endl;
				return false;
			}

			// Check camera type
			if (mConfigFile["camera"]["model"] == "virtual") {
				mStereoCamera = StereoCamera::create(StereoCamera::eModel::Virtual);
			}
			else if (mConfigFile["camera"]["model"] == "artec") {
				mStereoCamera = StereoCamera::create(StereoCamera::eModel::ArtecEva);
			}
			else if (mConfigFile["camera"]["model"] == "zed") {
				mStereoCamera = StereoCamera::create(StereoCamera::eModel::Zed);
			}
			else if (mConfigFile["camera"]["model"] == "http") {
				mStereoCamera = StereoCamera::create(StereoCamera::eModel::Http);
			}
			else if (mConfigFile["camera"]["model"] == "realSense") {
				mStereoCamera = StereoCamera::create(StereoCamera::eModel::RealSense);
			}
			else {
				std::cout << "[MAIN APPLICATION] Camera model not found" << std::endl;
			}

			if (mStereoCamera == nullptr)
				return false;

			if (!mStereoCamera->init(mConfigFile["camera"]["parameters"])) {
				std::cout << "[MAIN APPLICATION] Can't initialize the camera" << std::endl;
				return false;
			}

			// Init map
			mMap.init(mConfigFile["mapping"]);

			mBaseName = "cloud_"+std::to_string(time(NULL));

			// Init GUI
			Gui::init(2);
			Gui::get()->registerCallback([&](const pcl::visualization::KeyboardEvent &_event, void*_ptr) {
				if (_event.getKeySym() == "space" && _event.keyDown()) {
					// Stop the system.
					mRecording = !mRecording;
					if (mRecording) {
						mMap.init(mConfigFile["mapping"]);
						std::cout << "Start Recording!" << std::endl;
					}
					else
						std::cout << "Stop Recording!" << std::endl;
				}
				else if (_event.getKeySym() == "Return" && _event.keyUp()) {
					pcl::io::savePCDFileBinaryCompressed(mBaseName +" (" + std::to_string(mStoreIndex++) + ").pcd", mMap.map());
				}
				else if (_event.getKeySym() == "t" && _event.keyDown()) {
					mStop = !mStop;
				}
			});
			
			mPlatformPort = new serial::Serial("/dev/ttyUSB0", 115200);
			std::this_thread::sleep_for(std::chrono::seconds(3));
			while(mPlatformPort->available()){
				mPlatformPort->read();
			}

			if(mPlatformPort == nullptr || !mPlatformPort->isOpen()){
				std::cout << "Error opening serial port" << std::endl;
				return false;
			}else{
				mPlatformPort->write("F100\r\n");
				while(mPlatformPort->available()){
					mPlatformPort->read();
				}
				std::cout << "Initilized serial port" << std::endl;
			}

			return true;

		}

		//---------------------------------------------------------------------------------------------------------------------
		bool MainApplication::step() {
			if (mStop) {
				while (mStop) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}

			if(mPlatformPort == nullptr || !mPlatformPort->isOpen()){
				std::cout << "Error with serial port" << std::endl;
				return false;
			}

			// MovePlatform
			std::string code = "G1 X"+std::to_string(mCurrentAngle)+"\r\n";
			//std::cout << "Sending code: " << code << std::endl;
			size_t bytes_wrote = mPlatformPort->write(code);
			while(mPlatformPort->available()){
				mPlatformPort->read();
			}
			if(bytes_wrote != code.size()){
				std::cout << "Error with serial port" << std::endl;
			}

			mCurrentAngle += 30;
			if(mCurrentAngle >= 360) mCurrentAngle = 0;

            PointCloud<PointXYZRGBNormal> newCloud;
			// Get new data
			if (!stepData(newCloud)) {
				std::cout << "[MAIN APPLICATION] Something failed getting new data." << std::endl;
				return false;
			}

			// Remove invalid points.

			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(newCloud, newCloud, indices);
			pcl::removeNaNNormalsFromPointCloud(newCloud, newCloud, indices);

			// Plot cloud
			Gui::get()->clean(0);
			Gui::get()->showCloud(newCloud, "newCloud", 5, 0, 0);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			// Update map
			if (mRecording) {
				//if (!stepUpdateMap(newCloud)) {
				//	std::cout << "[MAIN APPLICATION] Something failed updating the map." << std::endl;
				//	return false;
				//}
			}
			return true;
		}

		//---------------------------------------------------------------------------------------------------------------------
		MainApplication::~MainApplication() {
			Gui::end();
			if (mStereoCamera != nullptr)
				delete mStereoCamera;
		}

		//---------------------------------------------------------------------------------------------------------------------
		bool MainApplication::stepData(PointCloud<PointXYZRGBNormal> &_cloud) {
			// Get dense point cloud
			if(!mStereoCamera->grab()){
				std::cout << "[MAIN APPLICATION] Something failed grabbing new data" << std::endl;
				return false;    // 666 Take care! this is only for these
			}
			if (!mStereoCamera->cloud(_cloud)) {
				std::cout << "[MAIN APPLICATION] Stereo camera don't provide point cloud" << std::endl;
				return false;
			}

			return true;
		}

		//---------------------------------------------------------------------------------------------------------------------
		bool MainApplication::stepUpdateMap(PointCloud<PointXYZRGBNormal> &_cloud) { 
			if (mMap.update(_cloud.makeShared())) {
				

				Gui::get()->clean(1);
				Gui::get()->showCloud(mMap.map(), "map", 5, 0, 1);
				return true;
			}
			else {
				return false;
			}
		}

		//---------------------------------------------------------------------------------------------------------------------
		bool MainApplication::filterDataCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud){


		}
	}	//	namespace fruitSearcher
}	//	namespace app
