#include <math.h>
#include <ros/package.h>  /// ros::package::getPath
#include <fstream>
#include <iostream>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/visualization/cloud_viewer.h>     /// pcl::visualization::PCLVisualizer
#include <pcl/common/transforms.h>  /// pcl::transformPointCloud


#include "ImageEst.h"

#include <dynamic_reconfigure/server.h>
#include <ilcc2/LocatorConfig.h>
#include <tf/tf.h>

using namespace std;
using namespace message_filters;

ImageEst::Ptr camera_model;

bool have_old_data;
cv::Mat image_old;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_old(new pcl::PointCloud<pcl::PointXYZI>);
cv::Mat image_now;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_now(new pcl::PointCloud<pcl::PointXYZI>);

double cloud_max_distance;

cv::Mat canvas;
pcl::visualization::PCLVisualizer::Ptr viewer;

void project_cloud_to_image(cv::Mat image,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            string win_name = "cur_image") {
  cv::Mat rectifyImage;
  cv::undistort(image, rectifyImage, camera_model->camK, camera_model->distort_param, camera_model->camK);

  /// 生成每个点的颜色
  double low = 0.1, high = cloud_max_distance;
  int counter = 0;
  for (unsigned int index = 0; index < cloud->size(); index++) {
    pcl::PointXYZI ptmp = cloud->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector2d Pcam;
    if (camera_model->spaceToPlane(Pw, Pcam, cloud_max_distance)) {
      int x = Pcam[0];
      int y = Pcam[1];

      // double h = (ptmp.intensity-inten_low)/(inten_high-inten_low)*255;
      double h = (Pw[2] - low) / (high - low) * 255;

      unsigned char r, g, b;
      camera_model->HSVtoRGB(h, 100, 100, &r, &g, &b);

      cv::circle(rectifyImage, cv::Point2d(x, y), 0.6, cv::Scalar(r, g, b), 2);
      counter++;
    }
  }

  if (have_old_data) {
    cv::destroyWindow("current_image (press 'space' key to select iamge)");
    
    int w = rectifyImage.cols/2;
    int h = rectifyImage.rows/2;
    cv::resize(rectifyImage, rectifyImage, cv::Size(w, h));

    if (win_name == "cur_image")
      rectifyImage.copyTo(canvas(cv::Rect(w, 0, w, h)));
    else 
      rectifyImage.copyTo(canvas(cv::Rect(0, 0, w, h)));

    cv::imshow("selcted_image | current_image", canvas);  
  } else {
    cv::imshow("current_image (press 'space' key to select iamge)", rectifyImage);
  }


  if (cv::waitKey(1) == ' ') {
    image_old = image_now.clone();
    cloud_old = cloud_now;
    have_old_data = true;
  }
}

void show_rgb_cloud(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  camera_model->get_rgb_cloud(image, cloud, cloud_max_distance, rgb_cloud);

  string id = "1"; 
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_view(rgb_cloud);
  viewer->removePointCloud(id);
  viewer->addPointCloud<pcl::PointXYZRGB> (rgb_cloud, rgb_view, id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
  viewer->spinOnce(5);
}

void callback_updateExtrinsic(ilcc2::LocatorConfig& config, uint32_t level) {
  double d2r = M_PI / 180.;
  tf::Quaternion quat;
  quat.setRPY(config.roll * d2r, config.pitch * d2r, config.yaw * d2r);
  Eigen::Quaterniond eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::Matrix3d R_lidar_to_cam = Eigen::Matrix3d::Identity();
  R_lidar_to_cam = eigen_quat.toRotationMatrix();
  Eigen::Vector3d p_lidar_to_cam(config.pos_x, config.pos_y, config.pos_z);

  camera_model->setRt(R_lidar_to_cam, p_lidar_to_cam);
  ROS_INFO("Reconfigure Request (rpyxyz): %.2f %.2f %.2f %.2f %.2f %.2f",
           config.roll, config.pitch, config.yaw, config.pos_x, config.pos_y,
           config.pos_z);
}

void callback_LidarCamMsg(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
                          const sensor_msgs::ImageConstPtr& msg_img) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg_pc, *input_cloud);
  image_now = cv_bridge::toCvCopy(msg_img, "bgr8")->image;

  if (have_old_data) {
    project_cloud_to_image(image_old, cloud_old, "old_image");
  }
  
  cloud_now = input_cloud;
  project_cloud_to_image(image_now, cloud_now);

  show_rgb_cloud(image_now, cloud_now);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calib_node");

  ros::NodeHandle nh;
  std::string package_path = ros::package::getPath("ilcc2");
  ROS_INFO("ilcc2 package at %s", package_path.c_str());

  std::string yaml_path, image_topic, lidar_topic;
  ros::NodeHandle nh_private("~");
  nh_private.param<double>("cloud_max_distance", cloud_max_distance, 5);
  nh_private.param<std::string>("yaml_path", yaml_path, "pointgrey.yaml");
  nh_private.param<std::string>("image_topic", image_topic, "/front_image");
  nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");

  camera_model = std::make_shared<ImageEst>(package_path + "/config/" + yaml_path);
  {
    double deg2rad = M_PI / 180.;
    tf::Quaternion tf_quat;
    tf_quat.setRPY(-90 * deg2rad, 0, 180 * deg2rad);
    Eigen::Quaterniond eigen_quat(tf_quat.w(), tf_quat.x(), tf_quat.y(),
                                  tf_quat.z());
    Eigen::Matrix3d R_lidar_to_cam = eigen_quat.matrix();
    Eigen::Vector3d p_lidar_to_cam(0, -0.3, 0);
    camera_model->setRt(R_lidar_to_cam, p_lidar_to_cam);
  }

  dynamic_reconfigure::Server<ilcc2::LocatorConfig> server;
  dynamic_reconfigure::Server<ilcc2::LocatorConfig>::CallbackType f;
  f = boost::bind(&callback_updateExtrinsic, _1, _2);
  server.setCallback(f);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 2);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic, 2);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback_LidarCamMsg, _1, _2));


  // view cloud
  viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("viewer"));
  viewer->setSize(1000, 600);
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(0,20,0,  0,-1,-0.5,   0,0,1);
  // view image
  canvas.create(camera_model->m_image_size.height/2, camera_model->m_image_size.width, CV_8UC3);

  ROS_INFO("waiting for lidar image topic %s %s", lidar_topic.c_str(), image_topic.c_str());

  while (ros::ok()) {
    ros::spin();
  }

  cout << "==========================\n\n";
  cout << "R lidar_to_cam:\n" << camera_model->m_R << "\n";
  cout << "P lidar_to_cam:\n" << camera_model->m_t.transpose() << "\n";
  cout << "\n==========================\n";

  return 0;
}
