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

#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

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
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "point_cloud_stitching/pcl_filter.hpp"
#include "point_cloud_utilities/pcl_utilities.hpp"
// #include "point_cloud_stitching/camera.hpp"

using namespace std;
using namespace filter;
using namespace pcl;
using namespace cv;

/************************************************/
//Global Variables..
/************************************************/
ros::Publisher pub;
const int MAX_FRAMES=60;



PclFilter::PclFilter(ros::NodeHandle& nh,vector<string>& topics)//CameraInfo, DepthImage, ColorImage
{  // Subscribe to point cloud
    K.resize(9);
    K=Eigen::VectorXd::Zero(9);
    point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &PclFilter::onReceivedRawPointCloud,this);
    info_sub_ = nh.subscribe(topics[0],1,&PclFilter::cameraInfoCb,this);
    depth_sub_ = nh.subscribe(topics[1],1,&PclFilter::depthImageCb,this);
    image_sub_ = nh.subscribe(topics[2],1,&PclFilter::colorImageCb,this);
    publish_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_filter_node/filtered_points", 1);
     // Advertise service to update the params
    capture_point_cloud_srv = nh.advertiseService("/pcl_filter_node/capture_point_cloud", &PclFilter::onCapturePointCloud, this);
    frame_count=0;
    capture=false;
    camera_done=false;
    color_done=false;
    depth_done=false;
}

bool PclFilter::onCapturePointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    res.success=true;
    std::cout<<"Capturing the Point Clouds..."<<std::endl;
    capture=true;
    return true;
}


void PclFilter::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  std::cout<<"In Camera Info Call Back"<<std::endl;
  if(msg==nullptr) return;
    for(int i=0;i<9;i++)
        K(i)=msg->K.at(i);
    fx=K[0],cx=K[2],fy=K[4],cy=K[5];
    camera_done=true;
    frame_id=msg->header.frame_id;
    info_sub_.shutdown();
}

void PclFilter::colorImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    color_image=cv_ptr->image.clone();
    // imshow("Color_Image",color_image);
    // waitKey(3);                                          // Wait for a keystroke in the window
    color_done=true;
}

void PclFilter::depthImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    depth_image=cv_ptr->image.clone();
    // imshow("Depth_Image",depth_image);
    // waitKey(3);
    //TODO: Capturing only consistent pixels..
    if(depth_done==false)
    {
      combined_depth_image = cv::Mat::zeros(depth_image.size(), depth_image.type());
      combined_depth_image_matrix.resize(depth_image.rows,depth_image.cols);
      combined_depth_image_matrix=Eigen::MatrixXd::Zero(depth_image.rows,depth_image.cols);
      useful_pixels.resize(depth_image.rows,depth_image.cols);
      useful_pixels=Eigen::MatrixXd::Zero(depth_image.rows,depth_image.cols);
      consistent_pixels.resize(depth_image.rows,depth_image.cols);
      consistent_pixels=Eigen::MatrixXd::Ones(depth_image.rows,depth_image.cols);
    }
    depth_done=true;
    if(capture)
    {
      if(frame_count<MAX_FRAMES)
      {
        for(int i=0;i<depth_image.rows;i++)
        {
          for(int j=0;j<depth_image.cols;j++)
          {
            if(depth_image.at<unsigned short>(i,j)==0 || depth_image.at<unsigned short>(i,j)!=depth_image.at<unsigned short>(i,j)) 
              {
                consistent_pixels(i,j)=0;
                continue;
              }
            combined_depth_image_matrix(i,j)=combined_depth_image_matrix(i,j)+depth_image.at<unsigned short>(i,j);
            useful_pixels(i,j)++;
          }
        }
        frame_count++;
      }
      else
      {
        for(int i=0;i<depth_image.rows;i++)
          for(int j=0;j<depth_image.cols;j++)
            if(useful_pixels(i,j)&&consistent_pixels(i,j))
              combined_depth_image.at<unsigned short>(i,j)=combined_depth_image_matrix(i,j)/useful_pixels(i,j);
       
        pcl::PointCloud<pcl::PointXYZRGB> cloud = PCLUtilities::makePointCloud(color_image,combined_depth_image,K,frame_id);
        std::cout<<"Made Point Cloud"<<std::endl;
        PCLUtilities::publishPointCloud<PointXYZRGB>(cloud,publish_cloud);
        std::cout<<"Point Cloud Published..."<<std::endl;
        std::cout<<"Points: "<<cloud.points.size()<<std::endl;
        combined_point_cloud = cloud;
        combined_depth_image = cv::Mat::zeros(depth_image.size(), depth_image.type());
        combined_depth_image_matrix.resize(depth_image.rows,depth_image.cols);
        combined_depth_image_matrix=Eigen::MatrixXd::Zero(depth_image.rows,depth_image.cols);
        useful_pixels.resize(depth_image.rows,depth_image.cols);
        useful_pixels=Eigen::MatrixXd::Zero(depth_image.rows,depth_image.cols);
        consistent_pixels=Eigen::MatrixXd::Ones(depth_image.rows,depth_image.cols);
        capture=false;
        frame_count=0;
      }
    }
}


void PclFilter::onReceivedRawPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle pnh("~");
    vector<string> params={"/camera/depth/camera_info","/camera/depth/image_rect_raw","/camera/color/image_raw"};
    // vector<string> params={"/camera/aligned_depth_to_color/camera_info","/camera/aligned_depth_to_color/image_raw","/camera/color/image_raw"};
    PclFilter pf(pnh,params);    
    ros::spin();
    return 0;
}

// void normalSpaceSampling(PointCloud<PointXYZ>::Ptr cloud,PointCloud<PointXYZ>& output)
// {
//   PointCloud<PointNormal>::Ptr cloud_normals (new PointCloud<PointNormal> ());

//   std::cout<<"Cloud Size..."<<cloud->points.size()<<std::endl;

//   // Compute surface normals and curvature
//   pcl::NormalEstimation<PointXYZ, PointNormal> norm_est;
//   pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
//   norm_est.setSearchMethod (tree);
//   norm_est.setKSearch (64);
  
//   norm_est.setInputCloud (cloud);
//   norm_est.compute (*cloud_normals);

//   std::vector<int> aux_indices;
//   removeNaNFromPointCloud (*cloud_normals, *cloud_normals, aux_indices);
//   removeNaNNormalsFromPointCloud (*cloud_normals, *cloud_normals, aux_indices);

//   NormalSpaceSampling<PointNormal, PointNormal> normal_space_sampling;
//   normal_space_sampling.setInputCloud (cloud_normals);
//   normal_space_sampling.setNormals (cloud_normals);
//   normal_space_sampling.setBins (4, 4, 4);
//   normal_space_sampling.setSeed (0);
//   normal_space_sampling.setSample (static_cast<unsigned int> (cloud_normals->size ()) / 4);

//   IndicesPtr walls_indices (new std::vector<int> ());
//   normal_space_sampling.filter (*walls_indices);
  


//   for(int i=0;i<walls_indices->size();i++)
//   {
//     output.push_back(PointXYZ(cloud_normals->points[walls_indices->at(i)].x,cloud_normals->points[walls_indices->at(i)].y,cloud_normals->points[walls_indices->at(i)].z));
//   }

// }
