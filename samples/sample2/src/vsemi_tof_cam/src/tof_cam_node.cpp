#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <thread>

#include "boost/endian/conversion.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

#include <flann/flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include <pcl/console/print.h>

#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/don.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <vsemi_tof_cam/vsemi_tof_camConfig.h>

#include "tof_camera_driver.h"
#include "camera_info.h"
#include "settings.h"
#include "tof_camera.h"
#include "tof_image.h"

using namespace std;

static string package_path = "";

static ros::Publisher cloud_scene_publisher;
static ros::Publisher image_depth_Publisher;
static ros::Publisher image_grayscale_Publisher;
static ros::Publisher image_amplitude_Publisher;

static Settings settings;

static std::string strFrameID = "sensor_frame"; 

static bool process_busy = false;

uint orientation = 0;

void updateConfig(vsemi_tof_cam::vsemi_tof_camConfig &config, uint32_t level)
{
	ROS_INFO("Update config ...");

	settings.hdr = static_cast<uint>(config.hdr);
	settings.mode = static_cast<uint>(config.mode);
	
	settings.image_type = static_cast<uint>(config.image_type);

	settings.frameRate = config.frame_rate;

	settings.ignoreSaturatedPoints = config.ignore_saturated_points;

	settings.automaticIntegrationTime = config.automatic_integration_time;

	settings.integrationTimeATOF1  = static_cast<uint>(config.integration_time_0);
	settings.integrationTimeATOF2  = static_cast<uint>(config.integration_time_1);
	settings.integrationTimeATOF3  = static_cast<uint>(config.integration_time_2);
	settings.integrationTimeATOF4  = static_cast<uint>(config.integration_time_3);
	settings.integrationTimeBTOF1  = static_cast<uint>(config.integration_time_4);
	settings.integrationTimeBTOF2  = static_cast<uint>(config.integration_time_5);

	settings.integrationTimeGray   = static_cast<uint>(config.integration_time_gray);

	settings.minAmplitude1 = static_cast<uint>(config.min_amplitude_0);
	settings.minAmplitude2 = static_cast<uint>(config.min_amplitude_1);
	settings.minAmplitude3 = static_cast<uint>(config.min_amplitude_2);
	settings.minAmplitude4 = static_cast<uint>(config.min_amplitude_3);
	settings.minAmplitude5 = static_cast<uint>(config.min_amplitude_4);

    	settings.offsetDistance = config.offset_distance;

	settings.roi_leftX   = static_cast<uint>(config.roi_left_x);
	settings.roi_topY    = static_cast<uint>(config.roi_top_y);
	settings.roi_rightX  = static_cast<uint>(config.roi_right_x);
	settings.roi_bottomY = static_cast<uint>(config.roi_bottom_y);

	settings.range   = static_cast<uint>(config.range);

	settings.roi_rightX  -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
	settings.roi_bottomY -= (settings.roi_bottomY - settings.roi_topY + 1) % 4;

	settings.pointCloudColor = static_cast<uint>(config.point_cloud_color);

	orientation = static_cast<uint>(config.orientation);

	settings.angle_x = config.angle_x;
	settings.angle_y = config.angle_y;

	settings.runVideo = true;
	settings.startStream = true;
	settings.updateParam = true;
}

void initialise()
{
	ros::NodeHandle nh("~");

	cloud_scene_publisher     = nh.advertise<sensor_msgs::PointCloud2>("cloud_scene", 1);
	image_depth_Publisher     = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_grayscale_Publisher = nh.advertise<sensor_msgs::Image>("image_grayscale", 1);
	image_amplitude_Publisher = nh.advertise<sensor_msgs::Image>("image_amplitude", 1);

	settings.runVideo = false;
	settings.updateParam = false;
}

void publish_cloud(ros::Publisher publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Time time) {

	pcl::PCLPointCloud2 cloud2;
	pcl::toPCLPointCloud2(*cloud, cloud2);

	sensor_msgs::PointCloud2 ros_msg_pointcloud2;
	pcl_conversions::fromPCL(cloud2, ros_msg_pointcloud2);

	ros_msg_pointcloud2.header.frame_id = strFrameID;
	ros_msg_pointcloud2.header.stamp      = time;

	publisher.publish(ros_msg_pointcloud2);
}

void publish_image(ros::Publisher publisher, cv::Mat image, ros::Time time) {

	sensor_msgs::Image ros_msg;
	ros_msg.header.frame_id = strFrameID;
	ros_msg.height = image.rows;
	ros_msg.width = image.cols;
	ros_msg.encoding = sensor_msgs::image_encodings::BGR8;
	ros_msg.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
	ros_msg.step = image.cols * image.elemSize();
	size_t size = ros_msg.step * image.rows;
	ros_msg.data.resize(size);

	if (image.isContinuous())
	{
		memcpy((char*)(&ros_msg.data[0]), image.data, size);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg.data[0]);
		uchar* cv_data_ptr = image.data;
		for (int i = 0; i < image.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg.step);
			ros_data_ptr += ros_msg.step;
			cv_data_ptr += image.step;
		}
	}

	ros_msg.header.stamp   = time;

	publisher.publish(ros_msg);
}

/**
* to process the 3D scene
*/
void process_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, cv::Mat depth_bgr, cv::Mat grayscale, cv::Mat amplitude) {

	if (process_busy) return;

	process_busy = true;

	ros::Time curTime = ros::Time::now();

	// clean up the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> scene_indices;
	pcl::removeNaNFromPointCloud(*scene, *scene_clean, scene_indices);

	// skip empty cloud
	if ((! scene_clean->empty()) && scene_clean->points.size() > 0) {
/*
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(scene_clean);
		sor.setMeanK(30);
		sor.setStddevMulThresh(1.0);
		sor.filter(*scene_filtered);
*/
		// scene transform if you know the camera pose
		//Eigen::Matrix3f rotation_matrix3f;
		//rotation_matrix3f = 
		//	  Eigen::AngleAxisf(0.55, Eigen::Vector3f::UnitX())
		//	* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
		//	* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
		//
		//Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
		//transform_affine3f.translation() << 0, 1.0, -1.0;
		//transform_affine3f.rotate(rotation_matrix3f);
		//pcl::transformPointCloud (*scene_filtered, *scene_filtered, transform_affine3f);

		publish_cloud(cloud_scene_publisher, scene_clean, curTime);

		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);
		amplitude.convertTo(amplitude, CV_8UC1);
		cvtColor(amplitude, amplitude, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
		publish_image(image_amplitude_Publisher, amplitude, curTime);

	}

	process_busy = false;
} 

/**
* to receive a frame
*/
void tof_image_received(std::shared_ptr<ToF_Image> tof_image)
{
	if (process_busy) return;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
	std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);

	cloud_scene->points.insert(cloud_scene->points.end(), pts.begin(), pts.end());

	cv::Mat depth_bgr(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr);
	cv::Mat grayscale(tof_image->height, tof_image->width, CV_8UC1, tof_image->data_grayscale);
	cv::Mat amplitude(tof_image->height, tof_image->width, CV_32F, tof_image->data_amplitude);

	cloud_scene->resize(tof_image->n_points);
	cloud_scene->width = tof_image->n_points;
	cloud_scene->height = 1;
	cloud_scene->is_dense = false;

	if (orientation == 1) {	
		cv::Mat depth_bgr_rotated(tof_image->width, tof_image->height, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat grayscale_rotated(tof_image->width, tof_image->height, CV_8UC1, cv::Scalar(0));
		cv::Mat amplitude_rotated(tof_image->width, tof_image->height, CV_32F, 0.0);

		cv::rotate(depth_bgr, depth_bgr_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
		cv::rotate(grayscale, grayscale_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
		cv::rotate(amplitude, amplitude_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

		Eigen::Matrix3f r;
		r = 
			  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(-3.14159265 * 0.5, Eigen::Vector3f::UnitZ());

		Eigen::Affine3f t = Eigen::Affine3f::Identity();
		t.translation() << 0, 0, 0;
		t.rotate(r);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_rotated(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud (*cloud_scene, *scene_rotated, t);

		process_scene(scene_rotated, depth_bgr_rotated.clone(), grayscale_rotated.clone(), amplitude_rotated.clone());
	} else {
		process_scene(cloud_scene, depth_bgr.clone(), grayscale.clone(), amplitude.clone());
	}
}

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

	unsigned int major = tof_camera_driver.getFirmwareMajor();
	unsigned int minor = tof_camera_driver.getFirmwareMinor();

	uint16_t chipID = tof_camera_driver.getChipID();
	uint16_t waferID = tof_camera_driver.getWaferID();

	std::cerr << "Firmware: " << major << "." << minor << std::endl;
	std::cerr << "Chip: " << waferID << "." << chipID << std::endl;

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
