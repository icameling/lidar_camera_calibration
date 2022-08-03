#include "ImageEst.h"
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  /// cv::undistort

bool ImageEst::getRectifyParam(std::string cam_yaml) {
  cv::FileStorage fs;
  fs.open(cam_yaml, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::cerr << "can not open " << cam_yaml << std::endl;
    return false;
  }
  fs["K"] >> camK;
  fs["d"] >> distort_param;
  m_image_size.width = static_cast<int>(fs["Camera.width"]);
  m_image_size.height = static_cast<int>(fs["Camera.height"]);
  fs.release();

  m_fx = camK.at<double>(0, 0);
  m_cx = camK.at<double>(0, 2);
  m_fy = camK.at<double>(1, 1);
  m_cy = camK.at<double>(1, 2);

  std::cout << "camK: \n" << camK << std::endl;
  std::cout << "dist: \n" << distort_param << std::endl;

  return true;
}

void ImageEst::undistort_image(cv::Mat image, cv::Mat &rectify_image) {
  cv::undistort(image, rectify_image, camK, distort_param, camK);
}

bool ImageEst::spaceToPlane(Eigen::Vector3d &P_w, Eigen::Vector2d &P_cam,
                            double dis) {
  Eigen::Vector3d P_c = m_R * P_w + m_t;
  //        Eigen::Vector3d P_c = P_w;
  P_w = P_c;

  if (P_c[2] < 0 || P_c[2] > dis) {
    return false;
  }

  // Transform to model plane
  double u = P_c[0] / P_c[2];
  double v = P_c[1] / P_c[2];

  P_cam(0) = m_fx * u + m_cx;
  P_cam(1) = m_fy * v + m_cy;

  if (P_cam(0) > 0 && P_cam(0) < m_image_size.width && P_cam(1) > 0 &&
      P_cam(1) < m_image_size.height)
    return true;
  else
    return false;
}

void ImageEst::HSVtoRGB(int h, int s, int v, unsigned char *r, unsigned char *g,
                        unsigned char *b) {
  // convert from HSV/HSB to RGB color
  // R,G,B from 0-255, H from 0-260, S,V from 0-100
  // ref http://colorizer.org/

  // The hue (H) of a color refers to which pure color it resembles
  // The saturation (S) of a color describes how white the color is
  // The value (V) of a color, also called its lightness, describes how dark the
  // color is

  int i;

  float RGB_min, RGB_max;
  RGB_max = v * 2.55f;
  RGB_min = RGB_max * (100 - s) / 100.0f;

  i = h / 60;
  int difs = h % 60;  // factorial part of h

  // RGB adjustment amount by hue
  float RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f;

  switch (i) {
    case 0:
      *r = RGB_max;
      *g = RGB_min + RGB_Adj;
      *b = RGB_min;
      break;
    case 1:
      *r = RGB_max - RGB_Adj;
      *g = RGB_max;
      *b = RGB_min;
      break;
    case 2:
      *r = RGB_min;
      *g = RGB_max;
      *b = RGB_min + RGB_Adj;
      break;
    case 3:
      *r = RGB_min;
      *g = RGB_max - RGB_Adj;
      *b = RGB_max;
      break;
    case 4:
      *r = RGB_min + RGB_Adj;
      *g = RGB_min;
      *b = RGB_max;
      break;
    default:  // case 5:
      *r = RGB_max;
      *g = RGB_min;
      *b = RGB_max - RGB_Adj;
      break;
  }
}

void ImageEst::get_rgb_cloud(cv::Mat image,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             double cloud_max_distance,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgbcloud) {
  cv::Mat rectifyImage;
  this->undistort_image(image, rectifyImage);

  for (unsigned int index = 0; index < cloud->size(); index++) {
    pcl::PointXYZI ptmp = cloud->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector2d Pcam;
    if (this->spaceToPlane(Pw, Pcam, cloud_max_distance)) {
      int x = Pcam[0];
      int y = Pcam[1];

      pcl::PointXYZRGB rgbpoint;
      rgbpoint.b = image.ptr<uchar>(y)[x * 3];
      rgbpoint.g = image.ptr<uchar>(y)[x * 3 + 1];
      rgbpoint.r = image.ptr<uchar>(y)[x * 3 + 2];
      rgbpoint.x = ptmp.x;
      rgbpoint.y = ptmp.y;
      rgbpoint.z = ptmp.z;

      rgbcloud->points.push_back(rgbpoint);
    }
  }
}
