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

#include "ilcc2/ImageCornersEst.h"
#include "ilcc2/config.h"

using namespace std;
using namespace message_filters;

ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);
ros::Publisher pubLaserCloud;
double distance_valid;

void publish_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud){

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*rgbcloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "/velodyne";
  pubLaserCloud.publish(cloud_msg);

}


void processData(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

  cv::Mat rectifyImage;
  image_corners_est->undistort_image(image, rectifyImage);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cout<<"start project "<< cloud->size() << " ,  ";
  for(unsigned int index=0; index<cloud->size(); index++){

    myPoint ptmp = cloud->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector2d Pcam;
    if ( image_corners_est->spaceToPlane(Pw, Pcam, distance_valid) ) {

      int x = Pcam[0];
      int y = Pcam[1];

      pcl::PointXYZRGB rgbpoint;
      rgbpoint.b = image.ptr<uchar>(y)[x*3];
      rgbpoint.g = image.ptr<uchar>(y)[x*3+1];
      rgbpoint.r = image.ptr<uchar>(y)[x*3+2];
      rgbpoint.x = ptmp.x;
      rgbpoint.y = ptmp.y;
      rgbpoint.z = ptmp.z;

      rgbcloud->points.push_back(rgbpoint);
    }
  }
  std::cout << image.channels() << std::endl;
  std::cout << rgbcloud->size() << " points ok\n";

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

  ros::init(argc,argv,"rgb_lidar");

  ros::NodeHandle nh;

  std::string package_path = ros::package::getPath("ilcc2");
  ROS_INFO("ilcc2 package at %s", package_path.c_str());


  std::string yaml_path, image_topic, lidar_topic;
  std::string extrinsic_file;
  ros::NodeHandle nh_private("~");

  nh_private.param<double>("distance_valid", distance_valid, 5);
  nh_private.param<std::string>("extrinsic_file", extrinsic_file, "/process_data_backup/pose.bin");
  nh_private.param<std::string>("yaml_path", yaml_path, "30w.yaml");
  nh_private.param<std::string>("image_topic", image_topic, "/front");
  nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");

  std::string extrinsic_path = package_path + extrinsic_file;
  ROS_INFO("load T_lidar2cam from %s", extrinsic_path.c_str());

  image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );
  image_corners_est->txt2extrinsic( extrinsic_path );

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/rgb_lidar", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 2);
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



