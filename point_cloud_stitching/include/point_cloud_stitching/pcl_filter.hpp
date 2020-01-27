#ifndef PCL_FILTER_HPP
#define PCL_FILTER_HPP

/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/PolygonMesh.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>


/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <Eigen/Core>

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

namespace filter
{
	/**
	* @brief The filter class deals with fusing the point clouds into one.
	*/
	class PclFilter
	{
	public:
		PclFilter(ros::NodeHandle& nh);

	private:
		void onReceivedRawPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in);
		// bool onGenerateMesh(point_cloud_stitching::GenerateMeshRequest& req, point_cloud_stitching::GenerateMeshResponse& res);
		// bool onReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
		// bool onUpdateParams(yak_ros_msgs::UpdateKinFuParamsRequest& req, yak_ros_msgs::UpdateKinFuParamsResponse& res);
		/** @brief Subscriber that listens to incoming point clouds */
		ros::Subscriber point_cloud_sub_;

		ros::Publisher publish_cloud;

		int frame_count;

		pcl::PointCloud<pcl::PointXYZ> combined_point_cloud;

	};
}  // namespace filter

#endif