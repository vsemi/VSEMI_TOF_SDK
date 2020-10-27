#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
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

#include "thread_safe.h"

using namespace std;

static string package_path = "";
static ros::Publisher cloud_scene_publisher;

static ros::Publisher markers_objects_publisher;
static ros::Publisher markers_cliff_publisher;
static ros::Publisher markers_safe_zone_publisher;

static ros::Publisher pose_camera_publisher;
static ros::Publisher marker_camera_publisher;

static ros::Publisher image_depth_Publisher;
static ros::Publisher image_grayscale_Publisher;

static Settings settings;

static std::string strFrameID = "sensor_frame"; 

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);

visualization_msgs::MarkerArray marker_array_objects;

static bool process_busy = false;

void updateConfig(vsemi_tof_cam::vsemi_tof_camConfig &config, uint32_t level)
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

	settings.runVideo = true;
	settings.startStream = true;
	settings.updateParam = true;
}

void initialise()
{
	ros::NodeHandle nh("~");
	nh.param("frame_rate", settings.frameRate, 50.0);

	cloud_scene_publisher     = nh.advertise<sensor_msgs::PointCloud2>("cloud_scene", 1);

	markers_objects_publisher = nh.advertise<visualization_msgs::MarkerArray>("markers_objects", 1);
	markers_cliff_publisher   = nh.advertise<visualization_msgs::MarkerArray>("markers_cliffs", 1);
	markers_safe_zone_publisher   = nh.advertise<visualization_msgs::MarkerArray>("markers_safe_zone", 1);

	pose_camera_publisher     = nh.advertise<geometry_msgs::PoseStamped>("pose_camera", 1);
	marker_camera_publisher     = nh.advertise<visualization_msgs::Marker>("marker_camera", 1);

	image_depth_Publisher     = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_grayscale_Publisher = nh.advertise<sensor_msgs::Image>("image_grayscale", 1);

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

void publish_markers(ros::Publisher publisher, std::vector<visualization_msgs::Marker> markers, ros::Time time) {
	
	visualization_msgs::MarkerArray marker_array;
	for (int i = 0; i < markers.size(); i ++) {
		visualization_msgs::Marker marker = markers[i];
		marker.header.stamp = time;
		marker_array.markers.push_back(marker);
	}

	publisher.publish(marker_array);
}

void analyze_obstacle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, ros::Time curTime) {

	std::vector<visualization_msgs::Marker> markers;

	visualization_msgs::Marker marker_deleteAll;
	marker_deleteAll.id =  0;
	marker_deleteAll.header.frame_id = strFrameID;
	marker_deleteAll.action = visualization_msgs::Marker::DELETEALL;
	markers.push_back(marker_deleteAll);

	if (scene->points.size() > 0) {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_pass(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (scene);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 1.5);
		pass.filter (*obstacle_pass);
		
		if (obstacle_pass->points.size() > 0) {

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud(obstacle_pass);
			sor.setMeanK(10);
			sor.setStddevMulThresh(1.0);
			sor.filter(*obstacle_filtered);

			if (obstacle_filtered->points.size() > 0) {
				pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
				feature_extractor.setInputCloud(obstacle_filtered);
				feature_extractor.compute();

				pcl::PointXYZRGB min_XYZ_;
				pcl::PointXYZRGB max_XYZ_;

				feature_extractor.getAABB(min_XYZ_, max_XYZ_);

				double width  = max_XYZ_.x - min_XYZ_.x; 
				double height = max_XYZ_.y - min_XYZ_.y; 
				double depth  = max_XYZ_.z - min_XYZ_.z;
				Eigen::Vector3f position(min_XYZ_.x + 0.5 * width, min_XYZ_.y + 0.5 * height, min_XYZ_.z + 0.5 * depth);

				visualization_msgs::Marker marker_cube;
				marker_cube.id = 1;
				marker_cube.header.frame_id = strFrameID;
		
				marker_cube.action = visualization_msgs::Marker::ADD;
				marker_cube.type = visualization_msgs::Marker::CUBE;
		
				marker_cube.pose.position.x = position[0];
				marker_cube.pose.position.y = position[1];
				marker_cube.pose.position.z = position[2];
				marker_cube.pose.orientation.x = 0.0;
				marker_cube.pose.orientation.y = 0.0;
				marker_cube.pose.orientation.z = 0.0;
				marker_cube.pose.orientation.w = 0.0;
		
				marker_cube.scale.x = width;
				marker_cube.scale.y = height;
				marker_cube.scale.z = 0.1;

				marker_cube.color.a = 0.5;

				marker_cube.color.r = 1.0;
				marker_cube.color.g = 0.0;
				marker_cube.color.b = 0.0;

				markers.push_back(marker_cube);

				visualization_msgs::Marker marker_label;
				marker_label.id =  2;
				marker_label.header.frame_id = strFrameID;
		
				marker_label.action = visualization_msgs::Marker::ADD;
				marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		
				marker_label.pose.position.x = 0;
				marker_label.pose.position.y = 0.8;
				marker_label.pose.position.z = 0.5;

				marker_label.pose.orientation.x = 0.0;
				marker_label.pose.orientation.y = 0.0;
				marker_label.pose.orientation.z = 0.0;
				marker_label.pose.orientation.w = 1.0;
		
				marker_label.scale.x = 0.15;
				marker_label.scale.y = 0.15;
				marker_label.scale.z = 0.15;

				marker_label.color.a = 0.85;

				marker_label.color.r = 1.0;
				marker_label.color.g = 0;
				marker_label.color.b = 0;

				std::stringstream stream_w, stream_h, stream_d, stream_dis;
				stream_w << std::fixed << std::setprecision(2) << width;
				stream_h << std::fixed << std::setprecision(2) << height;
				stream_d << std::fixed << std::setprecision(2) << depth;
				stream_dis << std::fixed << std::setprecision(2) << min_XYZ_.z; 

				marker_label.text = 
					"Obstacle detected! \nDistance: " + stream_dis.str() + 
					" meter: ";

				markers.push_back(marker_label);

			}
		}

	}

	publish_markers(markers_objects_publisher, markers, curTime);


}

void analyze_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, cv::Mat depth_bgr, cv::Mat grayscale) {

	if (process_busy) return;

	process_busy = true;

	ros::Time curTime = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> scene_indices;
	pcl::removeNaNFromPointCloud(*scene, *scene_clean, scene_indices);

	if ((! scene_clean->empty()) && scene_clean->points.size() > 0) {
		
		analyze_obstacle(scene_clean, curTime);
		
		geometry_msgs::PoseStamped pose_camera;

		pose_camera.header.frame_id = strFrameID;
		pose_camera.header.stamp = curTime;

		pose_camera.pose.position.x = 0;
		pose_camera.pose.position.y = 0;
		pose_camera.pose.position.z = 0;

		pose_camera.pose.orientation.x = 0;
		pose_camera.pose.orientation.y = 0;
		pose_camera.pose.orientation.z = 0;
		pose_camera.pose.orientation.w = 0;

		pose_camera_publisher.publish(pose_camera);

		publish_cloud(cloud_scene_publisher, scene_clean, curTime);

		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
	}

	process_busy = false;
}

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

	analyze_scene(scene, depth_bgr.clone(), grayscale.clone());
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
