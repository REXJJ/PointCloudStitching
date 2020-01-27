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

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>


#include <pcl/filters/filter.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <Eigen/Core>

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

#include "point_cloud_stitching/pcl_filter.hpp"
#include "point_cloud_utilities/pcl_utilities.hpp"

using namespace std;
using namespace filter;
using namespace pcl;

ros::Publisher pub;

const int MAX_FRAMES=1;

void cleanPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointNormal> & cleaned_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_voxelized;
    //voxelize the combined cloud so we have some semi clean data that isn't insanely large
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_in.makeShared());
    voxelSampler.setLeafSize(0.01,0.01,0.01);
    voxelSampler.filter(cloud_voxelized);

    // ROS_INFO("Performing MLS on the multisampled clouds...");
    // resample the data so we get normals that are reasonable using MLS
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // next perform MLS to average out the Z values and give us something reasonable looking
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_voxelized.makeShared());
    mls.setPolynomialOrder(3);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.01*10.0); // made this 1 order of magnitude bigger than the voxel size
    mls.setSqrGaussParam(mls.getSearchRadius()*mls.getSearchRadius()); // usually set this to be square of the search radius
    mls.process(mls_points);

    pcl::PointCloud<pcl::PointNormal> filtered_pcl;  
    //do a filter to get rid of any noisey points.
    // ROS_INFO("Removing statistical outliers...");
    // remove statistical outliers
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> statFilter;
    statFilter.setInputCloud(mls_points.makeShared());
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(0.2);
    // copy the filtered data into cloudfiltered
    statFilter.filter(filtered_pcl);

    cleaned_cloud = filtered_pcl;
}

PclFilter::PclFilter(ros::NodeHandle& nh)
{  // Subscribe to point cloud
    point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &PclFilter::onReceivedRawPointCloud,this);
    publish_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_stitch_node/filtered_points", 1);
    frame_count=0;
}

void PclFilter::onReceivedRawPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    // Convert to useful point cloud format
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    if(frame_count<MAX_FRAMES)
    {
      combined_point_cloud+=cloud;
      ++frame_count;
    }
    else
    {
      pcl::PointCloud<pcl::PointNormal> filtered_pcl;
      cleanPointCloud(combined_point_cloud,filtered_pcl);
      pcl::PCLPointCloud2* cloud_output = new pcl::PCLPointCloud2; 
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_output);
      pcl::toPCLPointCloud2(filtered_pcl, *cloud_output);
      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(*cloud_output, output);   
      output.is_bigendian = false;
      output.header.seq=1;
      output.header.stamp=ros::Time::now();
      output.header.frame_id=cloud_in->header.frame_id;
      output.height = filtered_pcl.height;
      output.width = filtered_pcl.width; 
      publish_cloud.publish (output);
      frame_count=0;
      combined_point_cloud.clear();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle pnh("~");
    PclFilter pf(pnh);    
    ros::spin();
    return 0;
}
