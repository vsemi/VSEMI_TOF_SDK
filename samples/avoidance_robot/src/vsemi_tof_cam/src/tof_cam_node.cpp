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

static bool automatic_camera_config = false;

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

	automatic_camera_config = config.automatic_camera_config;
}

void initialise()
{
	ros::NodeHandle nh("~");

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

struct CubeEigen {
	Eigen::Vector3f translation;
	Eigen::Quaternionf rotation;
	double width; double height; double depth;
	CubeEigen(Eigen::Vector3f t, 
		Eigen::Quaternionf r, 
		double w, double h, double d) : 
		translation(t), rotation(r), width(w), height(h), depth(d) {}
};

void cal_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _scene, pcl::PointCloud<pcl::Normal>::Ptr _normals) {

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne_scene;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_scene (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne_scene.setSearchMethod (tree_scene);
	ne_scene.setInputCloud (_scene);
	ne_scene.setKSearch (50);
	ne_scene.compute (*_normals);
}

Eigen::Vector3f camera_rotation_best;
Eigen::Vector3f camera_position_best;
float noise_level_best = 0.5;

Eigen::Vector3f camera_rotation;
Eigen::Vector3f camera_position;

float noise_level = 0.1;

bool camera_configured = false;
bool camera_valid_pose_detected = false;
int camera_configure_count = 0;

void detect_camera_pose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _scene) {

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane;
	seg_plane.setOptimizeCoefficients(true);
	seg_plane.setMethodType(pcl::SAC_RANSAC);
	seg_plane.setModelType(pcl::SACMODEL_PLANE);
	seg_plane.setDistanceThreshold(noise_level);
	seg_plane.setInputCloud(_scene);
	seg_plane.segment(*inliers_plane, *coefficients_plane);
	
	if (! camera_valid_pose_detected) {
		Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xz_plane_normal_vector;

		floor_plane_normal_vector[0] = coefficients_plane->values[0];
		floor_plane_normal_vector[1] = coefficients_plane->values[1];
		floor_plane_normal_vector[2] = coefficients_plane->values[2];

		xz_plane_normal_vector[0] = 0.0;
		xz_plane_normal_vector[1] = -1.0;
		xz_plane_normal_vector[2] = 0.0;

		Eigen::Vector3f rotation_vector = xz_plane_normal_vector.cross(floor_plane_normal_vector);
		float theta = -atan2(rotation_vector.norm(), xz_plane_normal_vector.dot(floor_plane_normal_vector));
		Eigen::AngleAxisf angleAxisf(theta, rotation_vector);

		Eigen::Quaternionf camera_quat(angleAxisf);

		camera_rotation = camera_quat.toRotationMatrix().eulerAngles(0, 1, 2);
	} else {
		float angle_x = camera_rotation[0] + 0.005;
		float angle_y = camera_rotation[1];
		float angle_z = camera_rotation[2];
		camera_rotation = Eigen::Vector3f(angle_x, angle_y, angle_z);
	}

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_floor(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.setInputCloud(_scene);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	extract.filter(*plane_floor);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(plane_floor);
	sor.setMeanK(30);
	sor.setStddevMulThresh(1.0);
	sor.filter(*plane_floor);

	Eigen::Matrix3f rotation_matrix3f;
	rotation_matrix3f = 
		  Eigen::AngleAxisf(camera_rotation[0], Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(camera_rotation[1], Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(camera_rotation[2], Eigen::Vector3f::UnitZ());

	Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
	transform_affine3f.translation() << 0, 0, 0;
	transform_affine3f.rotate(rotation_matrix3f);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*plane_floor, *floor_transformed, transform_affine3f);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
	feature_extractor.setInputCloud(floor_transformed);
	feature_extractor.compute();

	pcl::PointXYZRGB min_XYZ_;
	pcl::PointXYZRGB max_XYZ_;

	feature_extractor.getAABB(min_XYZ_, max_XYZ_);

	float _h = - 0.5 * (min_XYZ_.y + max_XYZ_.y);
	float noise_level = 0.5 * (max_XYZ_.y - min_XYZ_.y);
	camera_position = Eigen::Vector3f(0, _h, 0);

	camera_configure_count ++;

	if (noise_level < noise_level_best) {
		camera_rotation_best = camera_rotation;
		camera_position_best = camera_position;
		noise_level_best = noise_level;

		camera_valid_pose_detected = true;
	}

	if (camera_configure_count >= 60 && camera_valid_pose_detected) {

		std::string conf_file_name = package_path + "/data/camera.conf";


		ofstream cong_file;
		cong_file.open (package_path + "/data/camera.conf");
		cong_file << camera_rotation_best[0];
		cong_file << "\n";
		cong_file << camera_rotation_best[1];
		cong_file << "\n";
		cong_file << camera_rotation_best[2];
		cong_file << "\n";
		cong_file << camera_position_best[1];
		cong_file << "\n";
		cong_file << noise_level_best;
		cong_file.close();

		camera_rotation = camera_rotation_best;
		camera_position = camera_position_best;
		noise_level = noise_level_best;

		std::cout << "Saved camera config -" << std::endl;
		std::cout << "\ncamera_rotation:\n" << camera_rotation << std::endl;
		std::cout << "\ncamera_position:\n" << camera_position << std::endl;
		std::cout << "\nnoise_level: " << noise_level << std::endl;

		camera_configured = true;

		std::cout << "\nCamera pose auto-configured successfully!\n" << std::endl;
	}
}

void analyze_upper_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, ros::Time curTime) {

	std::vector<int> clean_scene_indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::removeNaNFromPointCloud(*scene, *clean_scene, clean_scene_indices);

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane;
	seg_plane.setOptimizeCoefficients(true);
	seg_plane.setMethodType(pcl::SAC_RANSAC);
	seg_plane.setModelType(pcl::SACMODEL_PLANE);
	seg_plane.setDistanceThreshold(noise_level);
	seg_plane.setInputCloud(clean_scene);
	seg_plane.segment(*inliers_plane, *coefficients_plane);
	
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.setInputCloud(clean_scene);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	extract.filter(*scene_plane);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_objects(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.setNegative(true);
	extract.filter(*scene_objects);

	std::vector<int> clean_plane_indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::removeNaNFromPointCloud(*scene_plane, *clean_plane, clean_plane_indices);

	pcl::PointXYZRGB plane_min_XYZ, plane_max_XYZ;

	pcl::PointXYZRGB plane_min_OBB;
	pcl::PointXYZRGB plane_max_OBB;
	pcl::PointXYZRGB plane_position_OBB;
	Eigen::Matrix3f plane_rotational_matrix_OBB;

	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_plane;
	feature_plane.setInputCloud(clean_plane);
	feature_plane.compute();

	feature_plane.getAABB(plane_min_XYZ, plane_max_XYZ);
	feature_plane.getOBB(plane_min_OBB, plane_max_OBB, plane_position_OBB, plane_rotational_matrix_OBB);

	Eigen::Vector3f plane_position(plane_position_OBB.x, plane_position_OBB.y, plane_position_OBB.z);
	Eigen::Quaternionf plane_quat(plane_rotational_matrix_OBB);
	double plane_width = plane_max_OBB.x - plane_min_OBB.x; 
	double plane_height = plane_max_OBB.y - plane_min_OBB.y; 
	double plane_depth = plane_max_OBB.z - plane_min_OBB.z;

	std::vector<int> clean_objects_indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_objects(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::removeNaNFromPointCloud(*scene_objects, *clean_objects, clean_objects_indices);

	if (clean_objects->points.size() > 0) {
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr objects_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		objects_tree->setInputCloud(clean_objects);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.05);
		ec.setMinClusterSize(10);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(objects_tree);
		ec.setInputCloud(clean_objects);
		ec.extract(cluster_indices);

		std::vector<visualization_msgs::Marker> markers;

		visualization_msgs::Marker marker_deleteAll;
		marker_deleteAll.id =  0;
		marker_deleteAll.header.frame_id = strFrameID;
		marker_deleteAll.action = visualization_msgs::Marker::DELETEALL;
		markers.push_back(marker_deleteAll);

		int i_object = 1;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cloud_cluster->points.push_back(clean_objects->points[*pit]);
			}

			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud(cloud_cluster);
			sor.setMeanK(30);
			sor.setStddevMulThresh(1.0);
			sor.filter(*cloud_cluster);

			std::vector<int> clean_cluster_indices;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::removeNaNFromPointCloud(*cloud_cluster, *clean_cluster, clean_cluster_indices);

			pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
			feature_extractor.setInputCloud(clean_cluster);
			feature_extractor.compute();

			pcl::PointXYZRGB min_XYZ_;
			pcl::PointXYZRGB max_XYZ_;
			pcl::PointXYZRGB min_OBB;
			pcl::PointXYZRGB max_OBB;
			pcl::PointXYZRGB position_OBB;
			Eigen::Matrix3f rotational_matrix_OBB;
			float major_value, middle_value, minor_value;
			Eigen::Vector3f major_vector, middle_vector, minor_vector;
			Eigen::Vector3f mass_center;

			feature_extractor.getAABB(min_XYZ_, max_XYZ_);
			feature_extractor.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);
			feature_extractor.getEigenValues(major_value, middle_value, minor_value);
			feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
			feature_extractor.getMassCenter(mass_center);

			if (max_XYZ_.y >= plane_max_XYZ.y) {

				Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
				Eigen::Quaternionf quat(rotational_matrix_OBB);
				double width = max_OBB.x - min_OBB.x; 
				double height = max_OBB.y - min_OBB.y; 
				double depth = max_OBB.z - min_OBB.z + (0.5 * noise_level);
				
				visualization_msgs::Marker marker_cube;
				marker_cube.id = 10 * i_object + 1;
				marker_cube.header.frame_id = strFrameID;
				
				marker_cube.action = visualization_msgs::Marker::ADD;
				marker_cube.type = visualization_msgs::Marker::CUBE;
				
				marker_cube.pose.position.x = position[0];
				marker_cube.pose.position.y = position[1];
				marker_cube.pose.position.z = position[2] - (0.5 * noise_level);
				marker_cube.pose.orientation.x = quat.x();
				marker_cube.pose.orientation.y = quat.y();
				marker_cube.pose.orientation.z = quat.z();
				marker_cube.pose.orientation.w = quat.w();
				
				marker_cube.scale.x = width;
				marker_cube.scale.y = height;
				marker_cube.scale.z = depth;

				marker_cube.color.a = 0.85;

				marker_cube.color.r = 1.0;
				marker_cube.color.g = 0.0;
				marker_cube.color.b = 0.0;

				markers.push_back(marker_cube);

				visualization_msgs::Marker marker_label;
				marker_label.id =  10 * i_object + 2;
				marker_label.header.frame_id = strFrameID;
				
				marker_label.action = visualization_msgs::Marker::ADD;
				marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				
				marker_label.pose.position.x = min_XYZ_.x;
				marker_label.pose.position.y = position[1] + 0.4;
				marker_label.pose.position.z = position[2];

				marker_label.pose.orientation.x = 0.0;
				marker_label.pose.orientation.y = 0.0;
				marker_label.pose.orientation.z = 0.0;
				marker_label.pose.orientation.w = 1.0;
				
				marker_label.scale.x = 0.050;
				marker_label.scale.y = 0.050;
				marker_label.scale.z = 0.050;

				marker_label.color.a = 0.85;

				marker_label.color.r = 1.0;
				marker_label.color.g = 0;
				marker_label.color.b = 0;

				std::stringstream stream_w, stream_h, stream_d, stream_dis;
				stream_w << std::fixed << std::setprecision(2) << width;
				stream_h << std::fixed << std::setprecision(2) << height;
				stream_d << std::fixed << std::setprecision(2) << depth;
				stream_dis << std::fixed << std::setprecision(2) << position[2];

				marker_label.text = 
					"obstacle:\nlength: " + stream_w.str() + 
					"\nwidth: " + stream_h.str() + 
					"\nheight: " + stream_d.str() + 
					"\ndistance: " + stream_dis.str();

				markers.push_back(marker_label);

			};

			i_object ++;
		}
		publish_markers(markers_objects_publisher, markers, curTime);
	}

	std::vector<visualization_msgs::Marker> markers;

	visualization_msgs::Marker marker_cube;
	marker_cube.id = 0;
	marker_cube.header.frame_id = strFrameID;
	
	marker_cube.action = visualization_msgs::Marker::ADD;
	marker_cube.type = visualization_msgs::Marker::CUBE;
	
	marker_cube.pose.position.x = plane_position[0];
	marker_cube.pose.position.y = plane_position[1];
	marker_cube.pose.position.z = plane_position[2];
	marker_cube.pose.orientation.x = plane_quat.x();
	marker_cube.pose.orientation.y = plane_quat.y();
	marker_cube.pose.orientation.z = plane_quat.z();
	marker_cube.pose.orientation.w = plane_quat.w();
	
	marker_cube.scale.x = plane_width;
	marker_cube.scale.y = plane_height;
	marker_cube.scale.z = 0.0001;

	marker_cube.color.a = 0.85;

	marker_cube.color.r = 0.0;
	marker_cube.color.g = 1.0;
	marker_cube.color.b = 0.0;

	markers.push_back(marker_cube);

	visualization_msgs::Marker marker_label;
	marker_label.id =  1;
	marker_label.header.frame_id = strFrameID;
	
	marker_label.action = visualization_msgs::Marker::ADD;
	marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	
	marker_label.pose.position.x = plane_max_XYZ.x; 
	marker_label.pose.position.y = plane_position[1] + 0.4;
	marker_label.pose.position.z = plane_position[2];

	marker_label.pose.orientation.x = 0.0;
	marker_label.pose.orientation.y = 0.0;
	marker_label.pose.orientation.z = 0.0;
	marker_label.pose.orientation.w = 1.0;
	
	marker_label.scale.x = 0.050;
	marker_label.scale.y = 0.050;
	marker_label.scale.z = 0.050;

	marker_label.color.a = 0.85;

	marker_label.color.r = 0;
	marker_label.color.g = 1.0;
	marker_label.color.b = 0;

	std::stringstream stream_w, stream_near, stream_far;
	stream_w << std::fixed << std::setprecision(2) << plane_width;
	stream_near << std::fixed << std::setprecision(2) << plane_min_XYZ.z;
	stream_far << std::fixed << std::setprecision(2) << plane_max_XYZ.z;

	marker_label.text = 
		"safe zone:\nwide: " + stream_w.str() + 
		"\nnear: " + stream_near.str() + 
		"\nfar: " + stream_far.str();

	markers.push_back(marker_label);

	publish_markers(markers_safe_zone_publisher, markers, curTime);
}

void analyze_lower_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, ros::Time curTime) {

	std::vector<int> clean_scene_indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::removeNaNFromPointCloud(*scene, *clean_scene, clean_scene_indices);

	if (clean_scene->empty() || clean_scene->points.size() == 0) return;

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	_tree->setInputCloud(clean_scene);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.03);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(_tree);
	ec.setInputCloud(clean_scene);
	ec.extract(cluster_indices);

	std::vector<visualization_msgs::Marker> markers;

	visualization_msgs::Marker marker_deleteAll;
	marker_deleteAll.id =  0;
	marker_deleteAll.header.frame_id = strFrameID;
	marker_deleteAll.action = visualization_msgs::Marker::DELETEALL;
	markers.push_back(marker_deleteAll);

	int i_cliff = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(clean_scene->points[*pit]);
		}

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud_cluster);
		sor.setMeanK(30);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_cluster);

		std::vector<int> clean_cluster_indices;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::removeNaNFromPointCloud(*cloud_cluster, *clean_cluster, clean_cluster_indices);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
		feature_extractor.setInputCloud(clean_cluster);
		feature_extractor.compute();

		pcl::PointXYZRGB min_XYZ_;
		pcl::PointXYZRGB max_XYZ_;
		pcl::PointXYZRGB min_OBB;
		pcl::PointXYZRGB max_OBB;
		pcl::PointXYZRGB position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getAABB(min_XYZ_, max_XYZ_);
		feature_extractor.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);

		if (min_XYZ_.y <= - (noise_level + 0.02)) {
			
			Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
			Eigen::Quaternionf quat(rotational_matrix_OBB);
			double width = max_OBB.x - min_OBB.x; 
			double height = max_OBB.y - min_OBB.y; 
			double depth = max_OBB.z - min_OBB.z;
			
			if (width > 0.1 && height > 0.1) {
				visualization_msgs::Marker marker_cube;
				marker_cube.id = 10 * i_cliff + 1;
				marker_cube.header.frame_id = strFrameID;
				
				marker_cube.action = visualization_msgs::Marker::ADD;
				marker_cube.type = visualization_msgs::Marker::CUBE;
				
				marker_cube.pose.position.x = position[0];
				marker_cube.pose.position.y = position[1];
				marker_cube.pose.position.z = position[2];
				marker_cube.pose.orientation.x = quat.x();
				marker_cube.pose.orientation.y = quat.y();
				marker_cube.pose.orientation.z = quat.z();
				marker_cube.pose.orientation.w = quat.w();
				
				marker_cube.scale.x = width;
				marker_cube.scale.y = height;
				marker_cube.scale.z = 0.0001;

				marker_cube.color.a = 0.85;

				marker_cube.color.r = 1.0;
				marker_cube.color.g = 0.0;
				marker_cube.color.b = 0.0;

				markers.push_back(marker_cube);

				visualization_msgs::Marker marker_label;
				marker_label.id =  10 * i_cliff + 2;
				marker_label.header.frame_id = strFrameID;
				
				marker_label.action = visualization_msgs::Marker::ADD;
				marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				
				marker_label.pose.position.x = max_XYZ_.x; 
				marker_label.pose.position.y = position[1] + 0.4;
				marker_label.pose.position.z = position[2];

				marker_label.pose.orientation.x = 0.0;
				marker_label.pose.orientation.y = 0.0;
				marker_label.pose.orientation.z = 0.0;
				marker_label.pose.orientation.w = 1.0;
				
				marker_label.scale.x = 0.050;
				marker_label.scale.y = 0.050;
				marker_label.scale.z = 0.050;

				marker_label.color.a = 0.85;

				marker_label.color.r = 1.0;
				marker_label.color.g = 0;
				marker_label.color.b = 0;

				std::stringstream stream_w, stream_h, stream_d, stream_dis;
				stream_w << std::fixed << std::setprecision(2) << width;
				stream_h << std::fixed << std::setprecision(2) << height;
				stream_d << std::fixed << std::setprecision(2) << min_XYZ_.y;
				stream_dis << std::fixed << std::setprecision(2) << position[2];

				marker_label.text = 
					"cliff:\nlength: " + stream_w.str() + 
					"\nwidth: " + stream_h.str() + 
					"\ndepth: " + stream_d.str() + 
					"\ndistance: " + stream_dis.str();

				markers.push_back(marker_label);
			}
		}

		i_cliff ++;
	}
	if (markers.size() > 0) {
		publish_markers(markers_cliff_publisher, markers, curTime);
	}
}

inline bool file_exists (const std::string& name) {
	struct stat buffer;
	return (stat (name.c_str(), &buffer) == 0);
}

void analyze_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, cv::Mat depth_bgr, cv::Mat grayscale) {

	if (process_busy) return;

	process_busy = true;

	ros::Time curTime = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> scene_indices;
	pcl::removeNaNFromPointCloud(*scene, *scene_clean, scene_indices);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(scene_clean);
	sor.setMeanK(30);
	sor.setStddevMulThresh(1.0);
	sor.filter(*scene_filtered);

	if ((! scene_filtered->empty()) && scene_filtered->points.size() > 100) {
		std::string conf_file_name = package_path + "/data/camera.conf";
		if (! camera_configured) {
			if (! file_exists(conf_file_name))
			{
				//std::cerr << "Camera pose config file is not available." << std::endl;
			} else {
				string line;
				ifstream ifs_(conf_file_name.c_str());
				getline(ifs_, line);
				float f1 = stof(line);
				getline(ifs_, line);
				float f2 = stof(line);
				getline(ifs_, line);
				float f3 = stof(line);
				getline(ifs_, line);
				float f4 = stof(line);
				getline(ifs_, line);
				float f5 = stof(line);
				camera_rotation = Eigen::Vector3f(f1, f2, f3);
				camera_position = Eigen::Vector3f(0, f4, 0);
				noise_level = f5;

				std::cout << "Loaded saved camera config -" << std::endl;
				std::cout << "\ncamera_rotation:\n" << camera_rotation << std::endl;
				std::cout << "\ncamera_position:\n" << camera_position << std::endl;
				std::cout << "\nnoise_level: " << noise_level << std::endl;

				camera_configured = true;
			}
		}

		if (automatic_camera_config || (! camera_configured)) {
			detect_camera_pose(scene_filtered);
		} 
		
		Eigen::Matrix3f rotation_matrix3f;
		rotation_matrix3f = 
			  Eigen::AngleAxisf(camera_rotation[0], Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(camera_rotation[1], Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(camera_rotation[2], Eigen::Vector3f::UnitZ());
	
		Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
		transform_affine3f.translation() << camera_position[0], camera_position[1], camera_position[2];
		transform_affine3f.rotate(rotation_matrix3f);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud (*scene_filtered, *scene_transformed, transform_affine3f);

		if (camera_configured && (! automatic_camera_config)) {

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_upper(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_lower(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PassThrough<pcl::PointXYZRGB> pass;
			pass.setInputCloud (scene_transformed);
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (-noise_level, 16.0); 
			pass.setFilterLimitsNegative (false);
			pass.filter (*scene_upper);
			pass.setFilterLimitsNegative (true);
			pass.filter (*scene_lower);

			analyze_lower_scene(scene_lower, curTime);
			analyze_upper_scene(scene_upper, curTime);
		}

		geometry_msgs::PoseStamped pose_camera;

		pose_camera.header.frame_id = strFrameID;
		pose_camera.header.stamp = curTime;

		pose_camera.pose.position.x = camera_position[0];
		pose_camera.pose.position.y = camera_position[1];
		pose_camera.pose.position.z = camera_position[2];

		Eigen::Quaternionf camera_quat(rotation_matrix3f);

		pose_camera.pose.orientation.x = camera_quat.x();
		pose_camera.pose.orientation.y = camera_quat.y();
		pose_camera.pose.orientation.z = camera_quat.z();
		pose_camera.pose.orientation.w = camera_quat.w();

		pose_camera_publisher.publish(pose_camera);

		visualization_msgs::Marker marker_camera;
		marker_camera.id =  0;
		marker_camera.header.frame_id = strFrameID;
		
		marker_camera.action = visualization_msgs::Marker::ADD;
		marker_camera.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		
		marker_camera.pose.position.x = camera_position[0];
		marker_camera.pose.position.y = camera_position[1];
		marker_camera.pose.position.z = camera_position[2];

		marker_camera.pose.orientation.x = 0.0;
		marker_camera.pose.orientation.y = 0.0;
		marker_camera.pose.orientation.z = 0.0;
		marker_camera.pose.orientation.w = 1.0;
		
		marker_camera.scale.x = 0.050;
		marker_camera.scale.y = 0.050;
		marker_camera.scale.z = 0.050;

		marker_camera.color.a = 0.85;

		marker_camera.color.r = 1.0;
		marker_camera.color.g = 1.0;
		marker_camera.color.b = 1.0;
		
		if (camera_configured) marker_camera.text = "Camera";
		else marker_camera.text = "Auto-configuring camera pose ...";

		publish_cloud(cloud_scene_publisher, scene_transformed, curTime);

		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
		marker_camera_publisher.publish(marker_camera);
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
