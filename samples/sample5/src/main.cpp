#include <iostream>

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <iterator>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "tof_camera_driver.h"

#include "thread_safe.h"

using namespace std;
using namespace cv;

static Settings settings;
cv::VideoCapture rgbVideo;
bool rgbVideo_ok;

void initConfig()
{
	settings.mode = 0; /*! 0 for wide FOV, and 1 for narrow beam */

	settings.hdr  = 2; /*! 0 HDR off, 1 for HDR spatial and 2 for HDR temporal */

	settings.frameRate = 50.0; /*! frame rate, this can be changed at runtime, for example 30.0 */

	settings.automaticIntegrationTime = false; /*! true for auto integration time, false to turn off auto integration time */

	settings.integrationTimeATOF1     = 500;      /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0, use this to set integration time for wide FOV */
	settings.integrationTimeATOF2     = 100;      /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	settings.integrationTimeATOF3     = 0;        /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	settings.integrationTimeATOF4     = 0;        /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */

	settings.minAmplitude1            = 50;       /*! 0 - 2047, threshold minAmplitude 0 beam A LSB */
	settings.minAmplitude2            = 10;       /*! 0 - 2047, threshold minAmplitude 1 beam A LSB */
	settings.minAmplitude3            = 0;        /*! 0 - 2047, threshold minAmplitude 2 beam A LSB */
	settings.minAmplitude4            = 0;        /*! 0 - 2047, threshold minAmplitude 3 beam A LSB */

	settings.roi_leftX   = 0;   /*! 0 - 160 (image width)  region of interest left   x */
	settings.roi_topY    = 0;   /*! 0 -  60 (image height) region of interest top    y */
	settings.roi_rightX  = 159; /*! 0 - 160 (image width)  region of interest right  x */
	settings.roi_bottomY = 59;  /*! 0 -  60 (image height) region of interest bottom y */

	settings.roi_rightX  -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
	settings.roi_bottomY -= (settings.roi_bottomY - settings.roi_topY + 1) % 4;

	settings.range  = 4000; /*! 0 - 9000, the depth range in mm, for coloring the BGR depth map and the 3D points */

	settings.startStream = true; /*! true to start ToF sensor */
	settings.runVideo    = true;    /*! true to start ToF sensor */
	settings.updateParam = true; /*! true to start ToF sensor */

	settings.ignoreSaturatedPoints = false; /*! true to ignore saturated points coursed by strong reflection or too big integration time */
}

void initialise()
{
	initConfig();
}

bool visualization_initialized = false;
bool update_visualize = false;
bool is_stopped = false;
std::recursive_mutex mutex_visualize_pcl;
std::recursive_mutex mutex_visualize_depth;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
cv::Mat *depth_bgr = new cv::Mat(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat *saturated_mask = new cv::Mat(60, 160, CV_8UC1, cv::Scalar(0));
cv::Mat *depth_mat = new cv::Mat(60, 160, CV_32F, 0.0f);
cv::Mat *m_rgb_frame = new cv::Mat(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));

/**
* PCL pointcloud visualization.
*/
void points_cloud_visualize() {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Vsemi TOF Camera"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(point_cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, colorHandler, "point_cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
	viewer->initCameraParameters();

	viewer->setCameraPosition(0, 0, -1,    0, 0, 0,   0, 0, 0);

	while (!viewer->wasStopped() && !is_stopped)
	{
		viewer->spinOnce(10);

		if (update_visualize) {
			synchronized(mutex_visualize_pcl) {
				viewer->removeAllShapes();
				viewer->removeAllPointClouds();
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(point_cloud_ptr);
				viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, colorHandler, "point_cloud");

				update_visualize = false;
			}
		}
	}
	is_stopped = true;
}

/**
* Obtain ToF_Image, constructing OpenCV Mat and PCL PointCloud for visualization.
*/
void update_tof_image_visualize(std::shared_ptr<ToF_Image> tof_image)
{
	synchronized(mutex_visualize_pcl) {
		point_cloud_ptr->points.clear();
		pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
		std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);
		point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());

		point_cloud_ptr->resize(tof_image->n_points);
		point_cloud_ptr->width = tof_image->n_points;
		point_cloud_ptr->height = 1;
		point_cloud_ptr->is_dense = false;

		update_visualize = true;
	}
	synchronized(mutex_visualize_depth) {
		*depth_mat = Mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_depth).clone();
		*depth_bgr = Mat(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr).clone();
		*saturated_mask = Mat(tof_image->height, tof_image->width, CV_8UC1, tof_image->saturated_mask).clone();
		resize(*depth_bgr, *depth_bgr, Size(), 3.0, 3.0);
	}
}

void update_rgb_frame()
{

	rgbVideo_ok = rgbVideo.open(1);
	if (rgbVideo_ok) {
		std::cout << "RGB video camera is ready!" << std::endl;
	} else {
		rgbVideo_ok = rgbVideo.open(0);
	if (rgbVideo_ok) {
		std::cout << "RGB video camera is ready!" << std::endl;
		} else {
			std::cout << "RGB video camera is not working!" << std::endl;
		}
	}

	while (!is_stopped)
	{
		rgbVideo >> *m_rgb_frame;
	}

	rgbVideo.release();
	
}

int main(int argc, char** argv) {

	cout << "Vsemi TOF Camera is starting ..." << endl;

	initialise();

	/**
	* Create driver instance, and init communication
	*/
	ToF_Camera_Driver tof_camera_driver(settings);

	bool initiated = tof_camera_driver.initCommunication();

	if (! initiated)
	{
		return 1;
	}

	std::thread th_update_rgb_frame(&update_rgb_frame);
	th_update_rgb_frame.detach();

	std::thread th_points_cloud_visualize(&points_cloud_visualize);
	th_points_cloud_visualize.detach();

	/**
	* Connect signal to obtain ToF_Imagem which contains data include raw distance data array, colored depth data array and 3D points data array.
	*/
	tof_camera_driver.sig_tof_image_received.connect(boost::bind(&update_tof_image_visualize, _1));

	while (!is_stopped)
	{
		/**
		* To request update, either requesting update parameters or requesting new frame
		*/
		tof_camera_driver.update();

		synchronized(mutex_visualize_depth) {
			cv::imshow("Depth Map BGR", *depth_bgr);

			if (! settings.ignoreSaturatedPoints) {
				cv::imshow("Saturated Mask", *saturated_mask);
			}
		}
	
		if (rgbVideo_ok)
		{
			cv::imshow("RGB Camera", *m_rgb_frame);
		}

		if (cv::waitKey(1) == 27)
		{
			is_stopped = true;
		}
	}

	cv::waitKey(100);

	return 0;
}
