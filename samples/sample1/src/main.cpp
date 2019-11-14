
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include "tof_camera_driver.h"

using namespace std;

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

/**
* Obtain ToF_Image, process data.
*/
void update_tof_image_visualize(std::shared_ptr<ToF_Image> tof_image)
{
	cout << "Image width: " << tof_image->width << " height: " << " points: " << tof_image->n_points << endl;
	// do something for the data below:
	//tof_image->data_3d_xyz_rgba;
	//tof_image->data_depth
	//tof_image->data_2d_bgr
	//data_3d_xyz_rgb
	for (int i = 0 ; i < tof_image->n_points; i ++)
	{
		float distance = tof_image->data_depth[i];
		cout << "     distance of point: " << i << ": " << distance << endl;
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
	tof_camera_driver.sig_tof_image_received.connect(boost::bind(&update_tof_image_visualize, _1));

	while (true)
	{
		/**
		* To request update, either requesting update parameters (in case settings changed at runtime) or requesting new frame
		*/
		tof_camera_driver.update();

		usleep(1);
	}

	return 0;
}
