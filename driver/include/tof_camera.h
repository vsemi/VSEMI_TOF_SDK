/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup tof_camera abstract ToF Camera
 * @brief abstract ToF_Camera
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_H_
#define TOF_CAMERA_H_

#include <boost/signals2.hpp>

#include "tof_image.h"

//! ToF_Camera
/*!
 * This abstract class is the low level interface of the ToF sensor used by the ToF_Camera_Driver.
 * An instance of ToF_Camera will be created by the ToF_Camera_Driver and stay live as long as you have the instance of ToF_Camera_Driver.
 */
class ToF_Camera{
public:

    /*!
    * @brief Initiate the communication to the sensor hardware.
    */
    virtual bool initCommunication() = 0;

	virtual unsigned int getFirmwareMajor() = 0;
	virtual unsigned int getFirmwareMinor() = 0;

	virtual uint16_t getChipID() = 0;
	virtual uint16_t getWaferID() = 0;

    /*!
    * @brief Request update to the sensor hardware, either sending new settings parameters, or request new frame.
    */
    virtual int update() = 0;

    /*!
    * @brief Signal to return ToF_Image back to ToF_Camera_Driver.
    */
    boost::signals2::signal<void (std::shared_ptr<ToF_Image>)> sig_tof_image_ready;

};

#endif // TOF_CAMERA_H_

/** @} */
