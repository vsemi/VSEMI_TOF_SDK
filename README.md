## VSEMI TOF SDK 1.3.6
Visionary Semiconductor Inc.
Jan 21, 2021

## Introduction

   **VSEMI ToF SDK** is the development kit for Sentinel-V3 ToF camera, developed by Visionary Semiconductor Inc.

   Built with the Time-of-Flight (**ToF**) depth vision technology, Sentinel-V3 was designed for reliable depth sensing with high accuracy. 
   It was developed for easy and fast development and/or deployment in real applications, such as robotics, drones, and automotives. 
   As a ToF 3D camera, **Sentinel-V3** does not require moving components and is thus robust and wear-free. 

## Quick start:

**Before start**
 - **Plug** your Vsemi ToF 3D sensor;
 - Grant **USB permission** to the current user (refer to the instruction of sample applications), and *remember that every time unplugged and plugged the sensor, need to re-grant USB permission*;
 - Compiling and running a **sample application** successfully is highly recommended before starting your own application;
 - **Dependencies**: **boost** is mandatory dependency for the ToF sensor driver, and additional dependencies depends on what you needed in your own application, for example if you need ROS, or OpenCV and PCL, and if configured properly in your development environment.

## Sample applications:

**[sample1](samples/sample1)**: work with raw data with minimum_dependency

**[sample2](samples/sample2)**: work with depth map and point cloud in ROS environment

Point cloud with distance pseudo-color:
![Image of Sample2](samples/sample2/sample_2_1.png)
 
Point cloud with grayscale:
![Image of Sample2](samples/sample2/sample_2_2.png)

**[sample3](samples/sample3)**:                   work with depth map and point cloud using OpenCV and PCL

The point cloud:
![Image of Sample3 = Point cloud](samples/sample3/sample_3_1.png)

## Developing you own application

 - **Driver**: copy the **include** files and **binary library** file into appropriate folder in your application development environment;
 - **Environment**: configure your application environment to make sure the **include** files in your **include** path and **binary library** file in your **link** path (you may refer to the sample applications how to configure it by using cmake);

 - **Dependencies**: determine dependencies, **boost** is the only mandatory dependency for the ToF sensor driver (you may refer to the [samples/sample1](samples/sample1), which the only dependency required is boost), and additional dependencies depends on what you needed in your own application, for example if you need ROS, or OpenCV and PCL, and configure them properly in your development environment.

## The basics
*For more information, refer to the sample application under [samples/sample1](samples/sample1)*

**settings**:
```
static Settings settings;

void initConfig()
{
	settings.mode = 0; /*! 0 for wide FOV, and 1 for narrow beam */

	settings.hdr  = 2; /*! 0 HDR off, 1 for HDR spatial and 2 for HDR temporal */

	settings.frameRate = 50.0; /*! frame rate, this can be changed at runtime, for example 30.0 */

	settings.automaticIntegrationTime = false; /*! true for auto integration time, false to turn off auto integration time */

	settings.integrationTimeATOF1     = 1000;      /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0, use this to set integration time for wide FOV */
	settings.integrationTimeATOF2     = 200;      /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	settings.integrationTimeATOF3     = 10;        /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	settings.integrationTimeATOF4     = 0;        /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */

	settings.minAmplitude1            = 10;       /*! 0 - 2047, threshold minAmplitude 0 beam A LSB */
	settings.minAmplitude2            = 20;       /*! 0 - 2047, threshold minAmplitude 1 beam A LSB */
	settings.minAmplitude3            = 60;        /*! 0 - 2047, threshold minAmplitude 2 beam A LSB */
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
```

**Subscribe to ToF camera image**:
```
void update_tof_image_visualize(std::shared_ptr<ToF_Image> tof_image)
{
	cout << "Image width: " << tof_image->width << " height: " << " points: " << tof_image->n_points << endl;
	// do something for the data below:
	//tof_image->data_3d_xyz_rgba;
	//tof_image->data_depth
	//tof_image->data_2d_bgr
	for (int i = 0 ; i < tof_image->n_points; i ++)
	{
		float distance = tof_image->data_depth[i];
		cout << "     distance of point: " << i << ": " << distance << endl;
	}
}
```

**The main function**:
```
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
```

## ROS
*For more information, refer to the sample application under [samples/sample2](samples/sample2)*

**Configuration**:
```
#!/usr/bin/env python

PACKAGE = "vsemi_tof_cam"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

op_mod_enum     = gen.enum([ gen.const("Wide_Field_image", int_t, 0, "Wide Field image"),
                             gen.const("Narrow_Field_manual", int_t, 1, "Narrow Field manual")
                             ], "An enum to set operating mode")

hdr_enum        = gen.enum([ gen.const("HDR_off", int_t, 0, "HDR off"),
                            gen.const("HDR_spatial", int_t, 1, "HDR spatial"),
                             gen.const("HDR_temporal", int_t, 2, "HDR temporal")
                           ], "An enum to set HDR modes")

color_enum        = gen.enum([ gen.const("Distance", int_t, 0, "Distance"),
                            gen.const("Grayscale", int_t, 1, "Grayscale")
                           ], "An enum to set Point Cloud Color")

######  Name                          Type   Reconfiguration level  Description     Default Min  Max
gen.add("mode",                       int_t,       0,  "Operating mode", 0, 0, 3, edit_method = op_mod_enum)

gen.add("angle_x",                    double_t,    0,  "Horizontal Angle of FOV [degrees]",  50.0,  0, 50.0 )
gen.add("angle_y",                    double_t,    0,  "Vertical Angle of FOV [degrees]",    18.75, 0, 18.75)

gen.add("frame_rate",                 double_t,    0,  "Frame rate [Hz]",  50.0, 0, 50.0)

gen.add("ignore_saturated_points",    bool_t,      0,  "Ignore Saturated Points",  False)
 
gen.add("hdr",                        int_t,       0,  "HDR mode", 2, 0, 2, edit_method = hdr_enum)

gen.add("automatic_integration_time", bool_t,      0,  "Automatic integration time",  False)

gen.add("integration_time_0",         int_t,       0,  "Integration time TOF for beam A [uS]",      1000, 0, 1000)
gen.add("integration_time_1",         int_t,       0,  "Integration time TOF for beam A [uS]",       200, 0, 1000)
gen.add("integration_time_2",         int_t,       0,  "Integration time TOF for beam A [uS]",        10, 0, 1000)
gen.add("integration_time_3",         int_t,       0,  "Integration time TOF for beam A [uS]",         0, 0, 1000)
gen.add("integration_time_4",         int_t,       0,  "Integration time TOF for beam B [uS]",         0, 0, 1000)
gen.add("integration_time_5",         int_t,       0,  "Integration time TOF for beam B [uS]",         0, 0, 1000)

gen.add("integration_time_gray",      int_t,       0,  "Integration time Grayscale [uS]",          25000, 0, 50000)

gen.add("min_amplitude_0",            int_t,       0,  "threshold minAmplitude 0 beam A LSB",         10, 0, 2047)
gen.add("min_amplitude_1",            int_t,       0,  "threshold minAmplitude 1 beam A LSB",         40, 0, 2047)
gen.add("min_amplitude_2",            int_t,       0,  "threshold minAmplitude 2 beam A LSB",         60, 0, 2047)
gen.add("min_amplitude_3",            int_t,       0,  "threshold minAmplitude 3 beam A LSB",          0, 0, 2047)
gen.add("min_amplitude_4",            int_t,       0,  "threshold minAmplitude 4 beam B LSB",          0, 0, 2047)

gen.add("offset_distance",            int_t,       0,  "distance offset mm",       0, -10000, 15000)

gen.add("roi_left_x",                 int_t,       0,  "ROI left X",     0, 0,  153)
gen.add("roi_top_y",                  int_t,       0,  "ROI top Y",      0, 0,   57)
gen.add("roi_right_x",                int_t,       0,  "ROI right X",  159, 5,  159)
gen.add("roi_bottom_y",               int_t,       0,  "ROI bottom Y",  59, 1,   59)

gen.add("point_cloud_color",          int_t,       0,  "Point Cloud Color", 0, 0, 1, edit_method = color_enum)

gen.add("range",                      int_t,       0,  "Range",  6000, 1000,   9000)

exit(gen.generate(PACKAGE, "tof_cam_node", "vsemi_tof_cam"))
```

**Subscribe to ToF image**:
```
/**
* to receive a frame
*/
void tof_image_received(std::shared_ptr<ToF_Image> tof_image)
{
	if (process_busy) return;

	cloud_scene->points.clear();

	pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
	std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);

	cloud_scene->points.insert(cloud_scene->points.end(), pts.begin(), pts.end());

	cv::Mat depth_bgr(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr);
	cv::Mat grayscale(tof_image->height, tof_image->width, CV_8UC1, tof_image->data_grayscale);

	cloud_scene->resize(tof_image->n_points);
	cloud_scene->width = tof_image->n_points;
	cloud_scene->height = 1;
	cloud_scene->is_dense = false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(*cloud_scene, *scene);

	process_scene(scene, depth_bgr.clone(), grayscale.clone());
}
```

**main loop**:
```
/**
* main
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tof_cam_node");

	dynamic_reconfigure::Server<vsemi_tof_cam::vsemi_tof_camConfig> server;
	dynamic_reconfigure::Server<vsemi_tof_cam::vsemi_tof_camConfig>::CallbackType f;
	f = boost::bind(&updateConfig, _1, _2);
	server.setCallback(f);

	package_path = ros::package::getPath("vsemi_tof_cam");

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
	tof_camera_driver.sig_tof_image_received.connect(boost::bind(&tof_image_received, _1));

	/**
	* ROS main loop
	*/
	while(ros::ok()){

		/**
		* To request update, either requesting update parameters or requesting new frame
		*/
		tof_camera_driver.update();

		ros::spinOnce();

	}
}
```

## Troubleshooting

**The most common problem is that the application is not able to connect to the sensor**, the reason might be:

 - The ToF 3D sensor not plugged;
 - USB permission not grant to current user;
 - The power USB provided is not enough so the sensor is not started correctly - although it is rare, but some edge devices or lightweight laptops, the USB power may not powerful enough and occasionally the sensor could not be started correctly, in such case, you may try unplug and plug in the sensor again and wait for a couple of more seconds and retry, and remember grant USB permission after re-plugged the sensor in.

## Contact

To purchase Vsemi ToF Camera, please visit https://vsemi.io

Contact us: info@vsemi.io

