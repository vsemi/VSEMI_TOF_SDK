/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup tof_image ToF_Image
 * @brief ToF_Image obtain from ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef VSEMI_TOF_IMAGE_H
#define VSEMI_TOF_IMAGE_H

#include <stdint.h>

//! ToF_Image
/*!
 * This struct defines ToF image obtained from ToF sensor, which contains data includes raw distance, colored 2D and 3D points array.
 * It can be obtained by subscribing to the signal sig_tof_image_received in ToF_Camera_Driver.
 */
struct ToF_Image
{
	int      width,      /*! Image width.    */
	         height,     /*! Image height.   */
	         n_points;   /*! Total points of the image. */

	/*!
	* @brief Pointer of an arry float[n_points], raw distance data in unit of meters.
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat depth_mat = Mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_depth);
	*/
	float*   data_depth;

	/*!
	* @brief Pointer of an arry float[n_points * 8], 3D points in XYZRGB format:
	* float x
	* float y
	* float z
	* float 0
	* *reinterpret_cast<float*>(&rgb), which rgb is type of uint32_t
	* *reinterpret_cast<float*>(&r),   which r   is type of uint8_t
	* *reinterpret_cast<float*>(&g),   which g   is type of uint8_t
	* *reinterpret_cast<float*>(&b),   which b   is type of uint8_t
	*
	* Example of reinterpreting to pcl PointCloud:
	*
	* pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	*
	* pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
	* std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);
	* point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
	*
	* point_cloud_ptr->resize(tof_image->n_points);
	* point_cloud_ptr->width = tof_image->n_points;
	* point_cloud_ptr->height = 1;
	* point_cloud_ptr->is_dense = false;
	*/
	float*   data_3d_xyz_rgb;

	/*!
	* @brief Pointer of an arry uint8_t[n_points * 3], 3-channel image in BGR format:
	* uint8_t b
	* uint8_t g
	* uint8_t r
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat depth_bgr = Mat(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr);
	*/
	uint8_t* data_2d_bgr;

	/*!
	* @brief Indices of saturated 3D points, use this to segment or remove saturated 3D points if needed.
	*
	* Only avaiable if ignoreSaturatedPoints in Settings set to false.
	*
	* Example of reinterpreting to pcl PointIndices:
	*
	* pcl::PointIndices saturatedPointIndices;
	* saturatedPointIndices.indices.insert(saturatedPointIndices.indices.end(), tof_image->saturated_indices.begin(), tof_image->saturated_indices.end());
	*/
	std::vector<int> saturated_indices;

	/*!
	* @brief Mask of saturated depth map, use this to segment or remove saturated 2D points from depth map if needed.
	*
	* Only avaiable if ignoreSaturatedPoints in Settings set to false.
	*
	* Example of reinterpreting to OpenCV Mat:
	*
	* Mat saturated_mask = Mat(tof_image->height, tof_image->width, CV_8UC1, tof_image->saturated_mask);
	*/
	uint8_t* saturated_mask;

	/*!
	* @param w: image width
	* @param h: image height
	*/
	ToF_Image(int w, int h) : width(w), height(h) {
		n_points            = w * h;
		data_depth          = new float[n_points];
		data_3d_xyz_rgb     = new float[n_points * 8];
		data_2d_bgr         = new uint8_t[n_points * 3];
		saturated_mask      = new uint8_t[n_points];
	}
};

#endif // VSEMI_TOF_IMAGE_H

/** @} */
