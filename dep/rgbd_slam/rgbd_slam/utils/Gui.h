////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_GUI_H_
#define RGBDSLAM_GUI_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include <thread>
#include <mutex>

namespace rgbd {
	/// Tool for visualization
	class Gui {
	public:		// Static interface
		static void init(unsigned _nViewports);
		static Gui *get();
		static void end();

	public:		// Public interface
		void showCloud(const pcl::PointCloud<pcl::PointXYZ> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);
		void showCloud(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);

		void showCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);
		void showCloud(const pcl::PointCloud<pcl::PointNormal> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);

		void showSurface(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, double _r = 1, double _g = 1, double _b = 1, unsigned _viewport = 0);
		void showSurface(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, unsigned _viewport = 0);

		void clean(unsigned _viewportIndex);
		void clean(std::string &_cloudName);

		bool hasReceivedStopSignal();

		void registerCallback(std::function<void(const pcl::visualization::KeyboardEvent &, void*)> _callback);

		void drawSphere(double _centroid[3], double _radius, std::string _name, unsigned _viewportIndex = 0);

		void drawEllipse(const double _semiAxis[3], const Eigen::Affine3d &_transformation, std::string _tag, unsigned _n = 10, double r = 1, double g = 1, double b = 1, unsigned _pointSize = 1, unsigned _viewport = 0);

		void drawArrow(const pcl::PointNormal &_point, std::string _tag, double _r = 1, double _g = 1, double _b = 1, unsigned _lineWidth = 1, unsigned _viewport = 0);


		template<typename FunctionType_, typename DataType_>
		void customPlot(FunctionType_ &_function, DataType_ &_data);

	private:	// Private methods
		Gui(unsigned _nViewports);
		~Gui();

		void callbackKeyboard3dViewer(const pcl::visualization::KeyboardEvent &_event, void* _ptrViewer);
		void displayThreadBody();

		template<typename PointType_>
		void addCloudToViewer(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);

		template<typename PointType_>
		void addCloudToViewerNormals(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);

		template<typename PointType_>
		void addSurface(const pcl::PointCloud<PointType_> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, double _r = 1, double _g = 1, double _b = 1, unsigned _viewport = 0);

		template<typename PointType_>
		void addSurface(const pcl::PointCloud<PointType_> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, unsigned _viewport = 0);

	private:	// Members
		boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
		std::vector<int> mViewportIndexes;

		std::thread *mDisplayThread;
		std::mutex	mSecureMutex;
		bool receivedStopSignal = false;

		std::vector<std::string>	mCloudsToClean;
		std::vector<int>			mViewportsToClean;

		std::function<void(const pcl::visualization::KeyboardEvent &, void*)> mUserCallback = [](const pcl::visualization::KeyboardEvent &, void*) {};

		struct DrawData {
			std::string mName;
			unsigned mPointSize;
			unsigned mViewport;
			int mNormals;
			double r, g, b, alpha;

		};

		typedef std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, DrawData>			DrawDataXYZ;
		typedef std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, DrawData>			DrawDataXYZRGB;
		typedef std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, DrawData>	DrawDataXYZRGBNormal;
		typedef std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, DrawData>			DrawDataNormal;
		typedef std::pair<pcl::PointNormal, DrawData>								DrawDataArrow;
		typedef std::pair<std::vector<pcl::Vertices>, DrawDataXYZ>					DrawDataSurface;
		typedef std::pair<std::vector<pcl::Vertices>, DrawDataXYZRGB>				DrawDataSurfaceRGB;
		typedef std::pair<pcl::ModelCoefficients, DrawData>							DrawDatashape;

		std::vector<DrawDataXYZ>			mQueueXYZ;
		std::vector<DrawDataXYZRGB>			mQueueXYZRGB;
		std::vector<DrawDataXYZRGBNormal>	mQueueXYZRGBNormal;
		std::vector<DrawDataNormal>			mQueueXYZNormal;
		std::vector<DrawDataArrow>			mQueueArrow;
		std::vector<DrawDataSurface>		mQueueSurfaces;
		std::vector<DrawDataSurfaceRGB>		mQueueSurfacesRGB;
		std::vector<DrawDatashape>			mQueueShapes;
		std::vector<std::function<void()>>	mQueueCustomDraw;

		static Gui *mInstance;
	};
}	//	namespace rgbd

#include "Gui.inl"

#endif	//	RGBDSLAM_GUI_H_
