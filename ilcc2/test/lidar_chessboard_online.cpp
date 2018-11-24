#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/package.h>        /// ros::package::getPath

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>    /// pcl::transformPointCloud

#include "ilcc2/LidarCornersEst.h"
#include "ilcc2/ImageCornersEst.h"
#include "ilcc2/Optimization.h"

using namespace std;
using namespace message_filters;

ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);
LidarCornersEst::Ptr lidar_corners_est(new LidarCornersEst);
ros::Publisher pubLaserCloud;
Eigen::Isometry3d T_board2img = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T_lidar2cam_axis_roughly = Eigen::Isometry3d::Identity();
double gray_zone_rate = 3;


void publish_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud){

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*rgbcloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "/velodyne";
  pubLaserCloud.publish(cloud_msg);

}

Eigen::Isometry3d get_lidar2cam_axis_roughly(std::string cam_name){

  Eigen::AngleAxisd R_lidarToCamera;
  if(cam_name == "front" || cam_name == "car_left")
    R_lidarToCamera = Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY())
                     *Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  if(cam_name == "left")
    R_lidarToCamera = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  if(cam_name == "right")
    R_lidarToCamera = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
                     *Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitZ());
  if(cam_name == "back")
    R_lidarToCamera = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY())
                     *Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());

  Eigen::Isometry3d T_lidar2cam_axis_roughly = Eigen::Isometry3d::Identity();
  T_lidar2cam_axis_roughly.rotate(R_lidarToCamera.matrix());

  return T_lidar2cam_axis_roughly;
}

void processData(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){


  if( image_corners_est->findCorners(image) ){
    Optimization optim;

    T_board2img = optim.solvePnP(image_corners_est->corners_now, image_corners_est->camK,
                                 image_corners_est->m_board_size, image_corners_est->m_grid_length);

  }

  Eigen::Vector3d t = T_board2img.translation();
  t = image_corners_est->T_lidar2cam.inverse() * t;
 // t = image_corners_est->T_lidar2cam.inverse() * T_lidar2cam_axis_roughly.inverse() * t;
  std::cout << "t: " << t.transpose() << std::endl;

  myPoint ptmp;
  ptmp.x = t(0);
  ptmp.y = t(1);
  ptmp.z = t(2);
  myPointCloud::Ptr chessboard_cloud(new myPointCloud);
  lidar_corners_est->get_chessboard_by_point(cloud, chessboard_cloud, ptmp);

//  pcl::PointXYZRGB rgb_point;
//  rgb_point.x =


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector2d gray_zone = lidar_corners_est->get_gray_zone(chessboard_cloud, gray_zone_rate);
  lidar_corners_est->color_by_gray_zone(chessboard_cloud, gray_zone, rgbcloud);

  publish_cloud(rgbcloud);

}


void callback_LidarCam(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
          const sensor_msgs::ImageConstPtr& msg_img)
{
//  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
//  ROS_INFO_STREAM("image received at " << msg_img->header.stamp.toSec());

  pcl::PointCloud<pcl::PointXYZI> input_cloud;

  pcl::fromROSMsg(*msg_pc, input_cloud);
  cv::Mat img = cv_bridge::toCvCopy(msg_img,"bgr8")->image;

  processData(img, input_cloud.makeShared());
}



int main(int argc, char** argv){

  ros::init(argc,argv,"pcd2image");

  ros::NodeHandle nh;

  std::string package_path = ros::package::getPath("ilcc2");
  ROS_INFO("ilcc2 package at %s", package_path.c_str());


  std::string yaml_path, image_topic, lidar_topic, camera_name;
  std::string extrinsic_file;
  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("extrinsic_file", extrinsic_file, "/process_data_backup/pose.bin");
  nh_private.param<std::string>("yaml_path", yaml_path, "30w.yaml");
  nh_private.param<std::string>("image_topic", image_topic, "/front");
  nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
  nh_private.param<std::string>("camera_name", camera_name, "front");
  nh_private.param<double>("gray_zone_rate", gray_zone_rate, 3.0);

  ROS_INFO("gray_zone_rate: %f", gray_zone_rate);

  T_lidar2cam_axis_roughly = get_lidar2cam_axis_roughly(camera_name);

  std::string extrinsic_path = package_path + extrinsic_file;
  ROS_INFO("load T_lidar2cam from %s", extrinsic_path.c_str());

  image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );
  image_corners_est->txt2extrinsic( extrinsic_path );

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/chessboard", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 5);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic, 5);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback_LidarCam, _1, _2));
  ROS_INFO("waiting for lidar image topic");


  while (ros::ok())
  {
    ros::spin();
  }
}



