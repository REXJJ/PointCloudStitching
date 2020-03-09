/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Trigger.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>


/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "point_cloud_stitching/pcl_filter.hpp"
#include "point_cloud_utilities/pcl_utilities.hpp"

using namespace std;
using namespace filter;
using namespace pcl;

/************************************************/
//Global Variables..
/************************************************/
ros::Publisher pub;
const int MAX_FRAMES=50;

void normalSpaceSampling(PointCloud<PointXYZ>::Ptr cloud,PointCloud<PointXYZ>& output)
{
  PointCloud<PointNormal>::Ptr cloud_normals (new PointCloud<PointNormal> ());

  std::cout<<"Cloud Size..."<<cloud->points.size()<<std::endl;

  // Compute surface normals and curvature
  pcl::NormalEstimation<PointXYZ, PointNormal> norm_est;
  pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (64);
  
  norm_est.setInputCloud (cloud);
  norm_est.compute (*cloud_normals);

  std::vector<int> aux_indices;
  removeNaNFromPointCloud (*cloud_normals, *cloud_normals, aux_indices);
  removeNaNNormalsFromPointCloud (*cloud_normals, *cloud_normals, aux_indices);

  NormalSpaceSampling<PointNormal, PointNormal> normal_space_sampling;
  normal_space_sampling.setInputCloud (cloud_normals);
  normal_space_sampling.setNormals (cloud_normals);
  normal_space_sampling.setBins (4, 4, 4);
  normal_space_sampling.setSeed (0);
  normal_space_sampling.setSample (static_cast<unsigned int> (cloud_normals->size ()) / 4);

  IndicesPtr walls_indices (new std::vector<int> ());
  normal_space_sampling.filter (*walls_indices);
  


  for(int i=0;i<walls_indices->size();i++)
  {
    output.push_back(PointXYZ(cloud_normals->points[walls_indices->at(i)].x,cloud_normals->points[walls_indices->at(i)].y,cloud_normals->points[walls_indices->at(i)].z));
  }

}


void cleanPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointNormal> & cleaned_cloud)
{   
    PointCloud<PointXYZ> output;
    const double leaf_size=0.01;
    // pcl::PointCloud<pcl::PointXYZ> cloud_voxelized;
    // //voxelize the combined cloud so we have some semi clean data that isn't insanely large
    // pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    // voxelSampler.setInputCloud(cloud_in.makeShared());
    // voxelSampler.setLeafSize(leaf_size,leaf_size,leaf_size);
    // voxelSampler.filter(cloud_voxelized);

    // normalSpaceSampling(cloud_voxelized.makeShared(),output);
    std::cout<<"Voxelized Cloud.."<<std::endl;

    // ROS_INFO("Performing MLS on the multisampled clouds...");
    // resample the data so we get normals that are reasonable using MLS
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // next perform MLS to average out the Z values and give us something reasonable looking
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_in.makeShared());
    mls.setPolynomialOrder(3);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(leaf_size*10.0); // made this 1 order of magnitude bigger than the voxel size
    mls.setSqrGaussParam(mls.getSearchRadius()*mls.getSearchRadius()); // usually set this to be square of the search radius
    mls.process(mls_points);

    pcl::PointCloud<pcl::PointNormal> filtered_pcl;  
    //do a filter to get rid of any noisey points.
    // ROS_INFO("Removing statistical outliers...");
    // remove statistical outliers
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> statFilter;
    statFilter.setInputCloud(mls_points.makeShared());
    statFilter.setMeanK(64);
    statFilter.setStddevMulThresh(0.2);
    // copy the filtered data into cloudfiltered
    statFilter.filter(filtered_pcl);

    cleaned_cloud = filtered_pcl;
}

PclFilter::PclFilter(ros::NodeHandle& nh)
{  // Subscribe to point cloud
    point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &PclFilter::onReceivedRawPointCloud,this);
    publish_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_filter_node/filtered_points", 1);
     // Advertise service to update the params
    capture_point_cloud_srv = nh.advertiseService("/pcl_filter_node/capture_point_cloud", &PclFilter::onCapturePointCloud, this);
    frame_count=0;
    capture=false;
}

bool PclFilter::onCapturePointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    res.success=true;
    std::cout<<"Capturing the Point Clouds..."<<std::endl;
    capture=true;
    return true;
}



void PclFilter::onReceivedRawPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    // Convert to useful point cloud format
    if(!capture)
      return;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
    if(frame_count<MAX_FRAMES)
    {
      combined_point_cloud+=cloud;
      const double leaf_size=0.01;
      pcl::PointCloud<pcl::PointXYZ> cloud_voxelized;
      //voxelize the combined cloud so we have some semi clean data that isn't insanely large
      pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
      voxelSampler.setInputCloud(combined_point_cloud.makeShared());
      voxelSampler.setLeafSize(leaf_size,leaf_size,leaf_size);
      voxelSampler.filter(cloud_voxelized);
      combined_point_cloud=cloud_voxelized;
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
      capture=false;
      std::cout<<"Point Cloud Captured and Send..."<<std::endl;
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
