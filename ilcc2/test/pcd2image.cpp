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

using namespace std;
using namespace message_filters;

ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);

double distance_valid;

void processData(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

  cv::Mat rectifyImage;
  cv::undistort(image, rectifyImage, image_corners_est->camK, image_corners_est->distort_param, image_corners_est->camK);

  pcl::PointXYZI ptmp;


  /// 生成每个点的颜色
  double inten_low=255, inten_high=0;
  std::vector<double> datas;
  for(unsigned int index=0; index<cloud->size(); index++){
      ptmp = cloud->points[index];
      if(inten_low > ptmp.intensity)
          inten_low = ptmp.intensity;
      if(inten_high < ptmp.intensity)
          inten_high = ptmp.intensity;
      datas.push_back(ptmp.intensity);
  }

  inten_low = 0;
  inten_high = 60;


//  std::cout<<"start project "<< cloud->size() << " ,  ";
  int counter = 0;
  for(unsigned int index=0; index<cloud->size(); index++){

    ptmp = cloud->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector2d Pcam;
    if ( image_corners_est->spaceToPlane(Pw, Pcam, distance_valid) ) {

      int x = Pcam[0];
      int y = Pcam[1];

      unsigned char r, g, b;

      double h = (ptmp.intensity-inten_low)/(inten_high-inten_low)*255;

      image_corners_est->HSVtoRGB(h, 100, 100, &r, &g, &b);

      cv::circle(rectifyImage, cv::Point2d(x,y), 0.6, cv::Scalar(r,g,b), 2);
//      cv::circle(rectifyImage, cv::Point2d(x,y), 0.8, cv::Scalar(r,g,b), 0.8);

//      image.ptr<uchar>(y)[x*3] = 255;
//      image.ptr<uchar>(y)[x*3+1] = 0;
//      image.ptr<uchar>(y)[x*3+2] = 0;

      counter++;
    }
  }
//  std::cout << counter << " points ok\n";
  cv::resize(rectifyImage, rectifyImage, cv::Size(rectifyImage.cols/1, rectifyImage.rows/1));
  cv::imshow("img_liar_point", rectifyImage);
  cv::waitKey(5);
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


  std::string yaml_path, image_topic, lidar_topic;
  std::string extrinsic_file;
  ros::NodeHandle nh_private("~");

  nh_private.param<double>("distance_valid", distance_valid, 5);
  nh_private.param<std::string>("extrinsic_file", extrinsic_file, "/process_data/pose.bin");
  nh_private.param<std::string>("yaml_path", yaml_path, "30w.yaml");
  nh_private.param<std::string>("image_topic", image_topic, "/front");
  nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");


  std::string extrinsic_path = package_path + extrinsic_file;
  ROS_INFO("load T_lidar2cam from %s", extrinsic_path.c_str());

  image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );

  image_corners_est->txt2extrinsic( extrinsic_path );

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 2);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic, 2);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback_LidarCam, _1, _2));
  ROS_INFO("waiting for lidar image topic %s %s", lidar_topic.c_str(), image_topic.c_str());


  while (ros::ok())
  {
    ros::spin();
  }
}



