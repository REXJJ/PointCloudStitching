/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

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
#include <pcl/filters/passthrough.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>


#include <pcl/filters/filter.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


#include <Eigen/Dense>
#include <Eigen/Core>

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

#include "point_cloud_stitching/pcl_fusion.hpp"
#include "point_cloud_utilities/pcl_utilities.hpp"

using namespace std;

using namespace fusion;

ros::Publisher pub;


static const std::double_t DEFAULT_MINIMUM_TRANSLATION = 0.1;

bool SAVE=false;
bool SAVING_DONE=false;

PclFusion::PclFusion(ros::NodeHandle& nh,const std::string& fusion_frame,vector<double>& box)
  : robot_tform_listener_(tf_buffer_)
  , fusion_frame_T_camera_prev_(Eigen::Affine3d::Identity())
  , fusion_frame_(fusion_frame)
  , bounding_box(box)
{  // Subscribe to point cloud
    point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &PclFusion::onReceivedPointCloud,this);
    saving_message_sub = nh.subscribe("/save_pcd_file", 1000, &PclFusion::chatterCallback,this);
    publish_cloud = nh.advertise<sensor_msgs::PointCloud2> ("fused_points", 1);
}

void PclFusion::onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    // Convert to useful point cloud format
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
    Eigen::Affine3d fusion_frame_T_camera = Eigen::Affine3d::Identity();
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    try
    {
        geometry_msgs::TransformStamped transform_fusion_frame_T_camera = tf_buffer_.lookupTransform(fusion_frame_, cloud_in->header.frame_id, ros::Time(0));
        fusion_frame_T_camera = tf2::transformToEigen(transform_fusion_frame_T_camera); 
    }
    catch (tf2::TransformException& ex)
    {
        // Abort integration if tf lookup failed
        ROS_WARN("%s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> temp;
    std::double_t motion_mag = (fusion_frame_T_camera.inverse() * fusion_frame_T_camera_prev_).translation().norm();//Get the distance traversed by the camera.
    if (motion_mag > DEFAULT_MINIMUM_TRANSLATION)
        ROS_WARN_STREAM(motion_mag);
    if (motion_mag < DEFAULT_MINIMUM_TRANSLATION)
    {
        ROS_DEBUG_STREAM("Camera motion below threshold");
        goto publishing;
    }
    pcl::transformPointCloud (cloud, cloud_transformed, fusion_frame_T_camera);
    for(auto point:cloud_transformed)
        if( point.z>bounding_box[4] && point.z<bounding_box[5] )
            temp.push_back(point);
    combined_pcl=combined_pcl+temp;//Combining the point clouds. TODO: Use ICP instead...
    combined_pcl=PCLUtilities::downsample(combined_pcl); 


    fusion_frame_T_camera_prev_=fusion_frame_T_camera;
    
    publishing: ; 

    temp.clear();
    for(auto point:combined_pcl)
        if(point.x>bounding_box[0] && point.x<bounding_box[1] && point.y>bounding_box[2] && point.y<bounding_box[3] && point.z>bounding_box[4] && point.z<bounding_box[5] )
            temp.push_back(point);

    combined_pcl=temp;

    if(SAVE&&!SAVING_DONE)
    {
        PCLUtilities::pclToXYZ<PointXYZ>(combined_pcl,"/home/rex/Desktop/REX_WORK_SPACE/Test_WS/REX/CGAL/Data/test.xyz");
        std::cout<<"Fusion Done..."<<std::endl;
    	SAVING_DONE=true;
    }

    // cout<<"SAVE : "<<SAVE<<" SAVING_DONE : "<<SAVING_DONE<<std::endl;
    
    pcl::PCLPointCloud2* cloud_output = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_output);
    pcl::toPCLPointCloud2(combined_pcl, *cloud_output);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_output, output);   
    output.is_bigendian = false;
    output.header.seq=1;
    output.header.stamp=ros::Time::now();
    output.header.frame_id=fusion_frame_;
    output.height = combined_pcl.height;
    output.width = combined_pcl.width; 
    publish_cloud.publish (output);
}

void PclFusion::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	// ROS_WARN("I heard: [%s]", msg->data.c_str());
	SAVE=true;
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void PclFusion::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
 
 }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_node");
    ros::NodeHandle pnh("~");
    string fusion_frame="";
    pnh.param<std::string>("fusion_frame", fusion_frame, "fusion_frame");
    std::vector<double> bounding_box;
    pnh.param("bounding_box", bounding_box, std::vector<double>());
    PclFusion pf(pnh,fusion_frame,bounding_box);    
    ros::spin();
    return 0;
}
