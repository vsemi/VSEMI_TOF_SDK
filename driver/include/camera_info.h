/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup camera_info CameraInfo
 * @brief CameraInfo of the ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_INFO_H_
#define TOF_CAMERA_INFO_H_

#include <stdint.h>
#include <vector>
#include <boost/array.hpp>

//! CameraInfo
/*!
 * This struct holds basic CameraInfo obtained from ToF sensor.
 */
struct RegionOfInterest_
{
    RegionOfInterest_()
    : x_offset(0)
    , y_offset(0)
    , height(0)
    , width(0)
    , do_rectify(false)
    {
    }

    typedef uint32_t _x_offset_type;
    uint32_t x_offset;

    typedef uint32_t _y_offset_type;
    uint32_t y_offset;

    typedef uint32_t _height_type;
    uint32_t height;

    typedef uint32_t _width_type;
    uint32_t width;

    typedef uint8_t _do_rectify_type;
    uint8_t do_rectify;
};

struct CameraInfo {

	   typedef uint32_t _height_type;
	  _height_type height;

	   typedef uint32_t _width_type;
	  _width_type width;

	   typedef std::vector<double>  _D_type;
	  _D_type D;

	   typedef boost::array<double, 9>  _K_type;
	  _K_type K;

	   typedef boost::array<double, 9>  _R_type;
	  _R_type R;

	   typedef boost::array<double, 12>  _P_type;
	  _P_type P;

      typedef RegionOfInterest_ _roi_type;
      _roi_type roi;
};

#endif /* TOF_CAMERA_INFO_H_ */

/** @} */
