#ifndef LIDARCORNERSEST_H_
#define LIDARCORNERSEST_H_

#include <iostream>
#include <memory>   /// std::shared_ptr
#include <pcl/common/common.h>
#include "ilcc2/config.h"

#include "ilcc2/Visualization.h"


class LidarCornersEst{

public:
  typedef std::shared_ptr<LidarCornersEst> Ptr;

  LidarCornersEst()
  {
    m_cloud_ROI = myPointCloud::Ptr(new myPointCloud);
    m_cloud_chessboard = myPointCloud::Ptr(new myPointCloud);
    m_cloud_PCA = myPointCloud::Ptr(new myPointCloud);
    m_cloud_optim = myPointCloud::Ptr(new myPointCloud);
    m_cloud_corners = myPointCloud::Ptr(new myPointCloud);
  }

  void register_viewer(void){


    visual_chessboard = Visualization();
    visual_chessboard.init_viewer("visual_chessboard");

    visual_corners = Visualization();
    visual_corners.init_viewer("visual_corners");

    visual_corners.register_keyboard("get_corner");
    visual_chessboard.register_keyboard("get_chessboard");
  }


  bool set_chessboard_param(std::string cam_yaml);
  void setROI(myPointCloud::Ptr cloud, myPoint point);

  bool get_chessboard_by_point(myPointCloud::Ptr incloud, myPointCloud::Ptr &outcloud, myPoint point);

  bool EuclideanCluster(void);
  myPointCloud::Ptr getPlane (myPointCloud::Ptr cloud);


  Eigen::Vector2d calHist(std::vector<double> datas);
  Eigen::Vector2d get_gray_zone(myPointCloud::Ptr cloud, double rate);
  Eigen::Matrix4f transformbyPCA(myPointCloud::Ptr input_cloud, myPointCloud::Ptr &output_cloud);

  void PCA(void);

  bool get_corners(std::vector<Eigen::Vector3d> &corners);
  void color_by_gray_zone(myPointCloud::Ptr cloud, Eigen::Vector2d gray_zone,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgbcloud);
  void getPCDcorners(Eigen::Matrix4f transPCA, Eigen::Matrix4f transOptim, myPointCloud &corners, bool inverse = false);
  void cornerCloud2vector(myPointCloud corner_cloud, std::vector<Eigen::Vector3d> &corners);


  myPoint m_click_point;
  myPointCloud::Ptr m_cloud_ROI;
  myPointCloud::Ptr m_cloud_chessboard;
  myPointCloud::Ptr m_cloud_PCA;
  myPointCloud::Ptr m_cloud_optim;
  myPointCloud::Ptr m_cloud_corners;


private:


  Eigen::Matrix4f pca_matrix;
  Eigen::Vector3f eigenValuesPCA;

  Eigen::Vector2d m_gray_zone;

  double m_grid_length;
  Eigen::Vector2i m_grid_size;                 // 指定标定板角点数


  Visualization visual_chessboard;
  Visualization visual_corners;
};




#endif
