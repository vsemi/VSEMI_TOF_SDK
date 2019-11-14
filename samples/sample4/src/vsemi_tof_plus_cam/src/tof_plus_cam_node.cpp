
#include "boost/endian/conversion.hpp"

#include <thread>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>

#include <vsemi_tof_plus_cam/vsemi_tof_plus_camConfig.h>

#include "tof_camera_driver.h"
#include "camera_info.h"
#include "settings.h"
#include "tof_camera.h"
#include "tof_image.h"

using namespace std;

/**
* Publisher to publish depth map and point cloud
*/
static ros::Publisher image_depth_Publisher;
static ros::Publisher image_saturated_mask_Publisher;
static ros::Publisher pointCloud2Publisher;
static ros::Publisher image_bgr_Publisher;

/**
* Settings to set parameters to communicate with Vsemi TOF sensor
*/
static Settings settings;
cv::VideoCapture rgbVideo;
bool rgbVideo_ok;
cv::Mat *m_rgb_frame = new cv::Mat(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
/**
* Frame ID
*/
static std::string strFrameID = "sensor_frame";
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

/**
* Bind the ROS config and the parameter settings
*/
void updateConfig(vsemi_tof_plus_cam::vsemi_tof_plus_camConfig &config, uint32_t level)
{
	ROS_INFO("Update config ...");

	settings.hdr = static_cast<uint>(config.hdr);
	settings.mode = static_cast<uint>(config.mode);

	settings.frameRate = config.frame_rate;

	settings.ignoreSaturatedPoints = config.ignore_saturated_points;

	settings.automaticIntegrationTime = config.automatic_integration_time;

	settings.integrationTimeATOF1  = static_cast<uint>(config.integration_time_0);
	settings.integrationTimeATOF2  = static_cast<uint>(config.integration_time_1);
	settings.integrationTimeATOF3  = static_cast<uint>(config.integration_time_2);
	settings.integrationTimeATOF4  = static_cast<uint>(config.integration_time_3);
	settings.integrationTimeBTOF1  = static_cast<uint>(config.integration_time_4);
	settings.integrationTimeBTOF2  = static_cast<uint>(config.integration_time_5);

	settings.minAmplitude1 = static_cast<uint>(config.min_amplitude_0);
	settings.minAmplitude2 = static_cast<uint>(config.min_amplitude_1);
	settings.minAmplitude3 = static_cast<uint>(config.min_amplitude_2);
	settings.minAmplitude4 = static_cast<uint>(config.min_amplitude_3);
	settings.minAmplitude5 = static_cast<uint>(config.min_amplitude_4);

	settings.roi_leftX   = static_cast<uint>(config.roi_left_x);
	settings.roi_topY    = static_cast<uint>(config.roi_top_y);
	settings.roi_rightX  = static_cast<uint>(config.roi_right_x);
	settings.roi_bottomY = static_cast<uint>(config.roi_bottom_y);

	settings.range   = static_cast<uint>(config.range);


	settings.roi_rightX  -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
	settings.roi_bottomY -= (settings.roi_bottomY - settings.roi_topY + 1) % 4;

	settings.runVideo = true;
	settings.startStream = true;
	settings.updateParam = true;
}

/**
* Initiate
*/
void initialise()
{
	ros::NodeHandle nh("~");
	nh.param("port_name", settings.port_name, std::string("/dev/ttyACM0"));
	nh.param("frame_rate", settings.frameRate, 50.0);

	image_depth_Publisher = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_saturated_mask_Publisher = nh.advertise<sensor_msgs::Image>("saturated_mask", 1);
	pointCloud2Publisher = nh.advertise<sensor_msgs::PointCloud2>("points", 1);
	image_bgr_Publisher = nh.advertise<sensor_msgs::Image>("image_bgr", 1);

	settings.runVideo = false;
	settings.updateParam = false;
}

/**
* Obtain ToF_Image, constructing OpenCV Mat and PCL PointCloud, and publish them.
*/
void publish_ros_messages(std::shared_ptr<ToF_Image> tof_image)
{
	point_cloud_ptr->points.clear();

	pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
	std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);

	point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());

	point_cloud_ptr->resize(tof_image->n_points);
	point_cloud_ptr->width = tof_image->n_points;
	point_cloud_ptr->height = 1;
	point_cloud_ptr->is_dense = false;

	cv::Mat depth_mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_depth);
	cv::Mat depth_bgr(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr);
	cv::Mat saturated_mask(tof_image->height, tof_image->width, CV_8UC1, tof_image->saturated_mask);
	cv::resize(depth_bgr, depth_bgr, cv::Size(), 3.0, 3.0);

	sensor_msgs::Image ros_msg_depth_bgr;
	ros_msg_depth_bgr.header.frame_id = strFrameID;
	ros_msg_depth_bgr.height = depth_bgr.rows;
	ros_msg_depth_bgr.width = depth_bgr.cols;
	ros_msg_depth_bgr.encoding = sensor_msgs::image_encodings::BGR8;
	ros_msg_depth_bgr.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
	ros_msg_depth_bgr.step = depth_bgr.cols * depth_bgr.elemSize();
	size_t size = ros_msg_depth_bgr.step * depth_bgr.rows;
	ros_msg_depth_bgr.data.resize(size);

	if (depth_bgr.isContinuous())
	{
		memcpy((char*)(&ros_msg_depth_bgr.data[0]), depth_bgr.data, size);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg_depth_bgr.data[0]);
		uchar* cv_data_ptr = depth_bgr.data;
		for (int i = 0; i < depth_bgr.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg_depth_bgr.step);
			ros_data_ptr += ros_msg_depth_bgr.step;
			cv_data_ptr += depth_bgr.step;
		}
	}

	sensor_msgs::Image ros_msg_saturated_mask;
	ros_msg_saturated_mask.header.frame_id = strFrameID;
	ros_msg_saturated_mask.height = saturated_mask.rows;
	ros_msg_saturated_mask.width = saturated_mask.cols;
	ros_msg_saturated_mask.encoding = sensor_msgs::image_encodings::MONO8;
	ros_msg_saturated_mask.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
	ros_msg_saturated_mask.step = saturated_mask.cols * saturated_mask.elemSize();
	size_t size_mask = ros_msg_saturated_mask.step * saturated_mask.rows;
	ros_msg_saturated_mask.data.resize(size_mask);

	if (saturated_mask.isContinuous())
	{
		memcpy((char*)(&ros_msg_saturated_mask.data[0]), saturated_mask.data, size_mask);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg_saturated_mask.data[0]);
		uchar* cv_data_ptr = saturated_mask.data;
		for (int i = 0; i < saturated_mask.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg_saturated_mask.step);
			ros_data_ptr += ros_msg_saturated_mask.step;
			cv_data_ptr += saturated_mask.step;
		}
	}

	pcl::PCLPointCloud2 cloud2;
	pcl::toPCLPointCloud2(*point_cloud_ptr, cloud2);

	sensor_msgs::PointCloud2 ros_msg_pointcloud2;
	pcl_conversions::fromPCL(cloud2, ros_msg_pointcloud2);

	ros_msg_pointcloud2.header.frame_id = strFrameID;

	ros::Time curTime = ros::Time::now();
	ros_msg_depth_bgr.header.stamp        = curTime;
	ros_msg_saturated_mask.header.stamp   = curTime;
	ros_msg_pointcloud2.header.stamp      = curTime;

	pointCloud2Publisher.publish(ros_msg_pointcloud2);
	image_depth_Publisher.publish(ros_msg_depth_bgr);
	image_saturated_mask_Publisher.publish(ros_msg_saturated_mask);

	if (rgbVideo_ok)
	{
		cv::Mat rgb_frame = *m_rgb_frame;

		sensor_msgs::Image ros_msg_rgb_frame;
		ros_msg_rgb_frame.header.frame_id = strFrameID;
		ros_msg_rgb_frame.height = rgb_frame.rows;
		ros_msg_rgb_frame.width = rgb_frame.cols;
		ros_msg_rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
		ros_msg_rgb_frame.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
		ros_msg_rgb_frame.step = rgb_frame.cols * rgb_frame.elemSize();
		size_t size = ros_msg_rgb_frame.step * rgb_frame.rows;
		ros_msg_rgb_frame.data.resize(size);

		if (rgb_frame.isContinuous())
		{
			memcpy((char*)(&ros_msg_rgb_frame.data[0]), rgb_frame.data, size);
		}
		else
		{
			uchar* ros_data_ptr = (uchar*)(&ros_msg_rgb_frame.data[0]);
			uchar* cv_data_ptr = rgb_frame.data;
			for (int i = 0; i < rgb_frame.rows; ++i)
			{
				memcpy(ros_data_ptr, cv_data_ptr, ros_msg_rgb_frame.step);
				ros_data_ptr += ros_msg_rgb_frame.step;
				cv_data_ptr += rgb_frame.step;
			}
		}
		ros_msg_rgb_frame.header.stamp        = curTime;
		image_bgr_Publisher.publish(ros_msg_rgb_frame);
		
	}
}

void update_rgb_frame()
{

	rgbVideo_ok = rgbVideo.open(1);
	if (rgbVideo_ok) {
		std::cout << "RGB video camera is ready!" << std::endl;
	} else {
		rgbVideo_ok = rgbVideo.open(0);
		if (rgbVideo_ok) {
			std::cout << "RGB video camera is ready!" << std::endl;
		} else {
			std::cout << "RGB video camera is not working!" << std::endl;
		}
	}

	while (ros::ok())
	{
		rgbVideo >> *m_rgb_frame;
	}
	
}

/**
* main
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "tof_plus_cam_node");

	dynamic_reconfigure::Server<vsemi_tof_plus_cam::vsemi_tof_plus_camConfig> server;
	dynamic_reconfigure::Server<vsemi_tof_plus_cam::vsemi_tof_plus_camConfig>::CallbackType f;
	f = boost::bind(&updateConfig, _1, _2);
	server.setCallback(f);

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

	std::thread th_update_rgb_frame(&update_rgb_frame);
	th_update_rgb_frame.detach();

	/**
	* Connect signal to obtain ToF_Imagem which contains data include raw distance data array, colored depth data array and 3D points data array.
	*/
	tof_camera_driver.sig_tof_image_received.connect(boost::bind(&publish_ros_messages, _1));

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

	if (rgbVideo_ok) {
		rgbVideo.release();
	}
}
