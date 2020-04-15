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
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
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
ros::Publisher data_collected;
const int MAX_FRAMES=60;
string file_name;
int take=0;
tf2_ros::Buffer tf_buffer_;

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
    ofstream f(file_name+"/transforms/camera.csv");
    f<<K(0);
    for(int i=1;i<9;i++)
      f<<","<<K(i);
    f.close();
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
      string depth_file=file_name+"/depth_images/image"+to_string(take);
      string color_file=file_name+"/color_images/image"+to_string(take);
       imwrite(color_file+".jpg",color_image);
       imwrite(depth_file+to_string(frame_count)+".png",depth_image);
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
 
        imwrite(file_name+"/depth_images/image_averaged"+to_string(take)+".png",depth_image);
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
        std_msgs::String msg;
        std::stringstream ss;
        ss << "All images saved... ";
        msg.data = ss.str();
        data_collected.publish(msg);
        //Saving robot transfromation.
        string robot_frame="/world";//Change it...
        string pcl_frame="/depth_optical_frame";
        try
        {
            geometry_msgs::TransformStamped transform_fusion_frame_T_camera = tf_buffer_.lookupTransform(robot_frame, pcl_frame, ros::Time(0));
            Eigen::Affine3d rob_T_cam = tf2::transformToEigen(transform_fusion_frame_T_camera);
            ofstream f(file_name+"/transforms/robot"+to_string(take)+".csv");
            for(int i=0;i<4;i++)
            {
              f<<rob_T_cam(i,0);
              for(int j=1;j<4;j++)
                f<<","<<rob_T_cam(i,j);
              f<<"\n";
            } 
        }
        catch (tf2::TransformException& ex)
        {
            // Abort integration if tf lookup failed
            ROS_WARN("%s", ex.what());
            return;
        }
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
    file_name = ros::package::getPath("point_cloud_stitching")+"/data";
    vector<string> params={"/camera/aligned_depth_to_color/camera_info","/camera/aligned_depth_to_color/image_raw","/camera/color/image_raw"};
    data_collected = pnh.advertise<std_msgs::String>("/data_collected",1);
    PclFilter pf(pnh,params);    
    ros::spin();
    return 0;
}

