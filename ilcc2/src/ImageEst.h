#ifndef IMAGECORNERSEST_H_
#define IMAGECORNERSEST_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core.hpp>

#include <memory>  /// std::shared_ptr

#include <fstream>

#include <pcl_conversions/pcl_conversions.h>

class ImageEst {
 public:
  typedef std::shared_ptr<ImageEst> Ptr;

  ImageEst(std::string cam_yaml) { getRectifyParam(cam_yaml); }

  bool getRectifyParam(std::string cam_yaml);

  void undistort_image(cv::Mat image, cv::Mat &rectify_image);

  void setRt(Eigen::Matrix3d R, Eigen::Vector3d t) {
    // T_lidar2cam
    m_R = R;
    m_t = t;
  }
  bool spaceToPlane(Eigen::Vector3d &P_w, Eigen::Vector2d &P_cam,
                    double dis = 6);

  void HSVtoRGB(int h, int s, int v, unsigned char *r, unsigned char *g,
                unsigned char *b);

  void get_rgb_cloud(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     double cloud_max_distance,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgbcloud);

  double m_fx, m_fy, m_cx, m_cy;
  cv::Mat camK, distort_param;

  cv::Size m_image_size;

  // lidar2cam
  Eigen::Matrix3d m_R;
  Eigen::Vector3d m_t;
};

#endif  // IMAGECORNERSEST_H_
