#ifndef PCL_FUSION_HPP
#define PCL_FUSION_HPP

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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

namespace fusion
{
	/**
	* @brief The fusion class deals with fusing the point clouds into one.
	*/
	class PclFusion
	{
	public:
		PclFusion(ros::NodeHandle& nh, const std::string& tsdf_frame,std::vector<double>& box);

	private:
		void onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in);
		void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
		// bool onGenerateMesh(point_cloud_stitching::GenerateMeshRequest& req, point_cloud_stitching::GenerateMeshResponse& res);
		// bool onReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
		// bool onUpdateParams(yak_ros_msgs::UpdateKinFuParamsRequest& req, yak_ros_msgs::UpdateKinFuParamsResponse& res);
		/** @brief Subscriber that listens to incoming point clouds */
		ros::Subscriber point_cloud_sub_;
		/** @brief Buffer used to store locations of the camera (*/
		tf2_ros::Buffer tf_buffer_;
		/** @brief Listener used to look up tranforms for the location of the camera */
		tf2_ros::TransformListener robot_tform_listener_;
		/** @brief Service that is called to trigger marching cubes and saving the mesh */
		ros::ServiceServer generate_mesh_service_;
		/** @brief Service that resets the tsdf volue */
		ros::ServiceServer reset_fusion_service_;
		/** @brief Service that updates the KinFuParams and resets the TSDF volume */
		ros::ServiceServer update_params_service_;
		/** @brief Used to track if the camera has moved. Only add image if it has */
		Eigen::Affine3d fusion_frame_T_camera_prev_;
		/** @brief TF frame associated with the TSDF volume. */
		std::string fusion_frame_;

		pcl::PointCloud<pcl::PointXYZ> combined_pcl;

		ros::Publisher publish_cloud;

		std::vector<double> bounding_box;

		ros::Subscriber saving_message_sub;

		void chatterCallback(const std_msgs::String::ConstPtr& msg);

	};
}  // namespace fusion

#endif