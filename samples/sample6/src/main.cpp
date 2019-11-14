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

#include "tof_camera_driver.h"

#include "thread_safe.h"

using namespace std;
using namespace cv;

static Settings settings;

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

cv::Mat *depth_bgr = new cv::Mat(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat *saturated_mask = new cv::Mat(60, 160, CV_8UC1, cv::Scalar(0));
cv::Mat *depth_mat = new cv::Mat(60, 160, CV_32F, 0.0f);

std::recursive_mutex mutex_visualize;

/**
* Obtain ToF_Image, reintepret into OpenCV Mat.
*/
void update_tof_image(std::shared_ptr<ToF_Image> tof_image)
{
	synchronized(mutex_visualize) {
		*depth_mat = Mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_depth).clone();
		*depth_bgr = Mat(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr).clone();
		*saturated_mask = Mat(tof_image->height, tof_image->width, CV_8UC1, tof_image->saturated_mask).clone();
		resize(*depth_bgr, *depth_bgr, Size(), 3.0, 3.0);
	}
}
int main() {

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

	/**
	* Connect signal to obtain ToF_Imagem which contains data include raw distance data array, colored depth data array and 3D points data array.
	*/
	tof_camera_driver.sig_tof_image_received.connect(boost::bind(&update_tof_image, _1));

	while (1)
	{
		/**
		* To request update, either requesting update parameters (in case settings changed at runtime) or requesting new frame
		*/
		tof_camera_driver.update();

		/**
		* Display the depth map
		*/
		synchronized(mutex_visualize) {
			cv::imshow("Depth Map BGR", *depth_bgr);

			/**
			* Display the saturated mask
			*/
			if (! settings.ignoreSaturatedPoints) {
				cv::imshow("Saturated Mask", *saturated_mask);
			}
		}

		if (cv::waitKey(1) == 27)
		{
			break;
		}
	}

	return 0;
}
