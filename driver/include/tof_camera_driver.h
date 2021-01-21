/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup tof_camera_driver ToF_Camera_Driver class
 * @brief class ToF_Camera_Driver
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_DRIVER_H_
#define TOF_CAMERA_DRIVER_H_

#include "settings.h"
#include "tof_camera.h"

//! ToF_Camera_Driver
/*!
 * This class is the interface between application and the ToF_Camera.
 * Application needs to create an instance of this class to communicate with the ToF sensor.
 */
class ToF_Camera_Driver
{

public:

	/*!
	* @brief ToF_Camera_Driver constructor
	*
	* @param set_, an instance of the struct Settings, then this instance of Settings will be singleton instance across the entire application,
	*  whenever the content of this singleton instance changed and immediately it will be sent to sensor hardware as soon as the ToF_Camera_Driver update method been called.
	*
	* @see Settings.
	*/
	ToF_Camera_Driver(
            std::string port_name_,
            uint  modulationFrequency_,
            bool  autoChannelEnabled_,
            int  channel_,
            Settings &set_);

	ToF_Camera_Driver(Settings &set_);

	~ToF_Camera_Driver();

	/*!
	* @brief Initiate the communication between ToF_Camera_Driver and ToF_Camera.
	*
	* @return true if the communication initiated successfully.
	*/
	bool initCommunication();

	unsigned int getFirmwareMajor();
	unsigned int getFirmwareMinor();

	uint16_t getChipID();
	uint16_t getWaferID();

	/*!
	* @brief Request update to the ToF sensor.
	* Either sending new settings parameters, or request new frame,
	* normally it could be called in the application main loop, for ROS, it could be called in the ROS loop, for example:
	*
	* while(ros::ok()){
	* 	tof_camera_driver.update();
	*
	* 	ros::spinOnce();
	* }
	*
	* @return true if updated successfully.
	*/
	int update();

	/*!
	* @brief Signal to obtain ToF_Image in the application, which contains depth and 3D points data.
	*
	* Example to subscribe to the signal to obtain ToF_Image:
	*
	* ToF_Camera_Driver tof_camera_driver(settings);
	*
	* bool initiated = tof_camera_driver.initCommunication();
	*
	* if (! initiated)
	* {
	* 	return 1;
	* }
	*
	* tof_camera_driver.sig_tof_image_received.connect(boost::bind(&update_tof_image_visualize, _1));
	*
	*/
	boost::signals2::signal<void (std::shared_ptr<ToF_Image>)> sig_tof_image_received;

private:
	ToF_Camera* tof_camera;

	unsigned int minor, major;
	uint16_t chipID, waferID;

	void tof_image_received(std::shared_ptr<ToF_Image> tof_image);
};

#endif // TOF_CAMERA_DRIVER_H_

/** @} */
