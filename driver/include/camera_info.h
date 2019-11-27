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


/**
* CameraInfo
*/
struct CameraInfo {

	typedef uint32_t _height_type;
	_height_type height;

	typedef uint32_t _width_type;
	_width_type width;

	/**
	* The distortion coefficient vector: (k1, k2, p1, p2, k3, k4, k5, k6), 
	*where k1 to k6 are for radial distortion, and the ps are for tangential distortion.
	* 
	* Example: -0.397059, 0.143377, 0, 0, 0, 0, 0, 0
	* 
	*/
	typedef std::vector<double>  _D_type;
	_D_type D;

	/**
	* Intrinsic camera matrix for the raw (distorted) images.
	*     [fx  0 cx]
	* K = [ 0 fy cy]
	*     [ 0  0  1]
	* Projects 3D points in the camera coordinate frame to 2D pixel
	* coordinates using the focal lengths (fx, fy) and principal point
	* (cx, cy)
	* 
	* Example:
	* 
	* 181.296 0         80
	* 0       181.079   30
	* 0       0         1
	* 
	*/
	typedef boost::array<double, 9>  _K_type;
	_K_type K;

	/**
	* Rectification matrix (stereo cameras only)
	* A rotation matrix aligning the camera coordinate system to the ideal
	* stereo image plane so that epipolar lines in both stereo images are
	* parallel.
	*/
	typedef boost::array<double, 9>  _R_type;
	_R_type R;

	/**
	* Projection/camera matrix
	*     [fx'  0  cx' Tx]
	* P = [ 0  fy' cy' Ty]
	*     [ 0   0   1   0]
	*/
	typedef boost::array<double, 12>  _P_type;
	_P_type P;

	/**
	* Region of interest (subwindow of full camera resolution)
	*/
	typedef RegionOfInterest_ _roi_type;
	_roi_type roi;
};

#endif /* TOF_CAMERA_INFO_H_ */

/** @} */
