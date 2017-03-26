////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_MAINAPPLICATION_H_
#define RGBDSLAM_MAINAPPLICATION_H_

#include <rgbd_slam/EnvironmentMap.h>
#include <rgbd_slam/StereoCamera.h>

#include <cjson/json.h>
#include <serial/serial.h>

namespace app {
	namespace ObjectScanner {
		/// \brief application that holds the main pipeline of the fruit searcher application
		class MainApplication {
		public:		// Public interface
			/// \brief This application only needs a path to a json configuration file. Configuration
			///	file needs following structure
			///	
			///	@code
			///		{
			///			"camera":
			///				{
			///					"model":"artec|virtual|zed|http|realSense",
			///					"parameters":
			///						{
			///							"See specific parameters in rgbd slam library"
			///						}
			///				},
			///			"mapping":
			///				{
            ///                 "msca":
            ///                 	{
            ///                 		"iterations":50,
            ///                 		"minScoreChange":0.000001,
            ///                 		"maxRotation":0.002,
            ///                 		"maxTranslation":0.001,
            ///                 		"correspondenceDistance":0.3,
            ///                 		"indexStaticCloud":0,
            ///                 		"samplingFactor":0.5,
			///							"queueSize":2
			///						},
			///					"voxelSize":0.001
			///				}
			///		}
			///	@endcode
			/// 	
			bool init(int _argc, char** _argv);

			/// \brief Main step of the aplication. It's mainly divided in two subs steps. Move the robot and acquiring the data.
			bool step();
			/// \brief Default destructor of class
			~MainApplication();

		private:
			bool stepData(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);
			bool stepUpdateMap(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);
			bool filterDataCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);


		private:	// Members
			rgbd::StereoCamera		*mStereoCamera = nullptr;
			rgbd::EnvironmentMap	mMap;
			bool	mRecording = false;
			bool	mStop = false;
			cjson::Json mConfigFile;
			unsigned mStoreIndex = 0;
			std::string mBaseName;

			serial::Serial *mPlatformPort;
			unsigned mCurrentAngle = 0;
		};	//	class MainApplication
	}	//	namespace fruitSearcher
}	//	namespace app

#endif	//	RGBDSLAM_MAINAPPLICATION_H_
