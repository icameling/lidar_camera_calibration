#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <iostream>
#include <Eigen/Core>

#include <pcl/visualization/cloud_viewer.h>     /// pcl::visualization::PCLVisualizer


#include "ilcc2/config.h"

class Visualization{


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Visualization() : m_confirm_flag(false), m_reject_flag(false)
  {
    m_plane_index = 0;
    update_flag = true;

    top_white_change = true;
    inverse_change = false;
  }
  void init_viewer(std::string name="viewer"){
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(name));
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(-12,0,5, 0, 0, 0, 0,0,0, 0);
    //viewer->setSize(1500, 1200);
  }

  void reset(void);

  void register_keyboard(std::string type)
  {
    if(type == "get_chessboard")
      viewer->registerKeyboardCallback(&Visualization::keyboard_get_chessboard, *this, 0);
    if(type == "get_corner")
      viewer->registerKeyboardCallback(&Visualization::keyboard_get_corner, *this, 0);
  }
  void keyboard_get_chessboard(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
  void keyboard_get_corner(const pcl::visualization::KeyboardEvent &event, void* viewer_void);


  void add_color_cloud(myPointCloud::Ptr cloud, Eigen::Vector3i color, std::string id, int size = 2);
  void add_rgb_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud, std::string id);
  void add_sphere_origin(void);

  void close_viewer(void);

  pcl::visualization::PCLVisualizer::Ptr viewer;

  bool m_confirm_flag;
  bool m_reject_flag;

  bool update_flag;
  unsigned int m_plane_index;

  bool inverse_change;
  bool top_white_change;

private:


  std::vector<pcl::PointIndices> cluster_indices;
  myPointCloud::Ptr m_cluster_cloud;

};



#endif //
