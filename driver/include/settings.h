/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup settings Settings
 * @brief Settings communicating between the driver and ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef VSEMI_TOF_DRIVER_SETTINGS_H
#define VSEMI_TOF_DRIVER_SETTINGS_H

#include <stdint.h>
#include <string>

//! Settings
/*!
 * This struct defines communicating parameters between the driver and ToF sensor.
 * Applications could use this to design control panel UI, for controlling ToF sensor at runtime.
 */
struct Settings{

	std::string port_name = "/dev/ttyACM0"; /*! USB port connects to the ToF sensor */

	bool startStream;  /*! true to start ToF sensor */
	bool runVideo;     /*! true to start ToF sensor */
	bool updateParam;  /*! true to start ToF sensor */

	double frameRate;  /*! frame rate, this can be changed at runtime, for example 30.0 */

	uint  mode;        /*! 0 for wide FOV, and 1 for narrow beam */

	uint  hdr;         /*! 0 HDR off, 1 for HDR spatial and 2 for HDR temporal */

	bool automaticIntegrationTime; /*! true for auto integration time, false to turn off auto integration time */

	uint integrationTimeATOF1;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0, use this to set integration time for wide FOV */
	uint integrationTimeATOF2;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeATOF3;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeATOF4;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeBTOF1;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 1, use this to set integration time for narrow beam */
	uint integrationTimeBTOF2;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 1, use this to set integration time for narrow beam */

	uint integrationTimeGray;      /*! 0 - 50000, integration time for grayscale */

	uint minAmplitude1;            /*! 0 - 2047, threshold minAmplitude 0 beam A LSB */
	uint minAmplitude2;            /*! 0 - 2047, threshold minAmplitude 1 beam A LSB */
	uint minAmplitude3;            /*! 0 - 2047, threshold minAmplitude 2 beam A LSB */
	uint minAmplitude4;            /*! 0 - 2047, threshold minAmplitude 3 beam A LSB */
	uint minAmplitude5;            /*! 0 - 2047, threshold minAmplitude 4 beam B LSB */

	uint roi_leftX;    /*! 0 - 160 (image width)  region of interest left   x */
	uint roi_topY;     /*! 0 -  60 (image height) region of interest top    y */
	uint roi_rightX;   /*! 0 - 160 (image width)  region of interest right  x */
	uint roi_bottomY;  /*! 0 -  60 (image height) region of interest bottom y */

	int offsetDistance;         /*! 0 - 15000, the depth offset in mm */

	int range;                  /*! 0 - 9000, the depth range in mm, for coloring the BGR depth map and the 3D points */

	bool ignoreSaturatedPoints; /*! true to ignore saturated points coursed by strong reflection or too big integration time */

	double angle_x      = 50.0f;  /*! horizontal angle of the FOV, modify this to calibrate horizontal angle of the FOV */
	double angle_y      = 18.75f; /*! vertical angle of the FOV, modify this to calibrate vertical angle of the FOV */

	uint pointCloudColor = 0; /*! point cloud color scheme, 0 - distance pseudo color, 1 - grayscale */

};

#endif // VSEMI_TOF_DRIVER_SETTINGS_H

/** @} */
