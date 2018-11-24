#include "ilcc2/LidarCornersEst.h"

#include <pcl/filters/passthrough.h>        /// pcl::PassThrough

/// EuclideanCluster
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/common/transforms.h>              /// pcl::transformPointCloud
#include <pcl/visualization/cloud_viewer.h>     /// pcl::visualization::PCLVisualizer

#include "ilcc2/Optimization.h"

#include <boost/thread/thread.hpp>

bool LidarCornersEst::set_chessboard_param(std::string cam_yaml)
{
  cv::FileStorage fs;
  fs.open(cam_yaml, cv::FileStorage::READ);

  if ( !fs.isOpened() )
  {
      std::cerr << "can not open " << cam_yaml << std::endl;
      return false;
  }
  m_grid_length = static_cast<double>(fs["grid_length"]);
  m_grid_size(0) = static_cast<int>(fs["corner_in_x"]) + 1;
  m_grid_size(1) = static_cast<int>(fs["corner_in_y"]) + 1;
  fs.release();

  if(m_grid_size(0) > m_grid_size(1)){
    int temp = m_grid_size(0);
    m_grid_size(0) = m_grid_size(1);
    m_grid_size(1) = temp;
  }

  std::cout << "grid_length: " << m_grid_length << std::endl;
  std::cout << "grid_in_x: " << m_grid_size(0) << std::endl;
  std::cout << "grid_in_y: " << m_grid_size(1) << std::endl;

  return true;
}

void LidarCornersEst::setROI(myPointCloud::Ptr cloud, myPoint point)
{
  pcl::PassThrough<myPoint> pass;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (point.z-2.0, point.z+2.0);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (point.x-1.0, point.x+1.0);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (point.y-1.5, point.y+1.5);
  pass.filter (*cloud);

  m_click_point = point;
  m_cloud_ROI = cloud;

}

bool LidarCornersEst::get_chessboard_by_point(myPointCloud::Ptr incloud, myPointCloud::Ptr &outcloud, myPoint point)
{
    pcl::search::KdTree<myPoint>::Ptr tree (new pcl::search::KdTree<myPoint>);
    tree->setInputCloud (incloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<myPoint> ec;   //欧式聚类对象
    ec.setClusterTolerance (0.1);                     // 设置近邻搜索的搜索半径为0.1m
    ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);                    //设置点云的搜索机制
    ec.setInputCloud (incloud);
    ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    tree->nearestKSearch(point, 1, k_indices, k_sqr_distances);

    unsigned int plane_index = 0;
    bool find_board = false;
    for(unsigned int i = 0; i < cluster_indices.size(); i++)
    {
      int counter = std::count(cluster_indices.at(i).indices.begin(), cluster_indices.at(i).indices.end(), k_indices.at(0));

      if(counter > 0)
      {
        plane_index = i;
        find_board = true;
        break;
      }
    }

    myPointCloud::Ptr temp_cloud (new myPointCloud);

    pcl::copyPointCloud<myPoint> (*incloud, cluster_indices.at(plane_index).indices, *temp_cloud);    /// 取出所有的内点
    temp_cloud = getPlane(temp_cloud);    /// 取出平面

    outcloud = temp_cloud;

    if(outcloud->size() < 500 || find_board == false)
      return false;

    return true;
}

/**
 * @brief LidarCornersEst::EuclideanCluster
 *
 *          input : m_cloud_ROI
 *          output: m_cloud_chessboard
 * @return
 */
bool LidarCornersEst::EuclideanCluster()
{
  pcl::search::KdTree<myPoint>::Ptr tree (new pcl::search::KdTree<myPoint>);
  tree->setInputCloud (m_cloud_ROI);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<myPoint> ec;   //欧式聚类对象
  ec.setClusterTolerance (0.1);                     // 设置近邻搜索的搜索半径为0.1m
  ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
  ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);                    //设置点云的搜索机制
  ec.setInputCloud (m_cloud_ROI);
  ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中


  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  tree->nearestKSearch(m_click_point, 1, k_indices, k_sqr_distances);

  visual_chessboard.reset();
  for(unsigned int i = 0; i < cluster_indices.size(); i++)
  {
    int counter = std::count(cluster_indices.at(i).indices.begin(), cluster_indices.at(i).indices.end(), k_indices.at(0));

    if(counter > 0)
    {
      visual_chessboard.m_plane_index = i;
      break;
    }
  }

  visual_chessboard.add_color_cloud(m_cloud_ROI, Eigen::Vector3i(255, 255, 255), "cloud_ROI");

  myPointCloud::Ptr temp_cloud (new myPointCloud);
  unsigned int plane_index = 0;
  while(visual_chessboard.m_confirm_flag == false) {

    if (visual_chessboard.update_flag) {

        visual_chessboard.update_flag = false;
        if(visual_chessboard.m_plane_index >=0 && visual_chessboard.m_plane_index < cluster_indices.size())
          plane_index = visual_chessboard.m_plane_index;

        pcl::copyPointCloud<myPoint> (*m_cloud_ROI, cluster_indices.at(plane_index).indices, *temp_cloud);    /// 取出所有的内点
        temp_cloud = getPlane(temp_cloud);    /// 取出平面
        visual_chessboard.add_color_cloud(temp_cloud, Eigen::Vector3i(238, 0, 255), "chess_plane");

        std::cout << "o: confirm;  r: change /click_point; w: plane_idx++; s: plane_idx--;  now plane_inx = " << plane_index << std::endl;
    }
    visual_chessboard.viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  visual_chessboard.viewer->removeAllPointClouds();

  if(visual_chessboard.m_reject_flag){
    std::cout << "change /click_point ..." << std::endl;
    return false;
  }

  m_cloud_chessboard = temp_cloud;
  std::cout << "chessboard plane size: " << m_cloud_chessboard->size() << std::endl;
  return true;
}



myPointCloud::Ptr LidarCornersEst::getPlane(myPointCloud::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<myPoint> seg;    /// Create the segmentation object
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);    // m

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

  //  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //                                      << coefficients->values[1] << " "
  //                                      << coefficients->values[2] << " "
  //                                      << coefficients->values[3] << std::endl;

    myPointCloud::Ptr final (new myPointCloud);

    pcl::copyPointCloud<myPoint> (*cloud, *inliers, *final);    /// 取出所有的内点

    return final;
}


Eigen::Vector2d LidarCornersEst::calHist(std::vector<double> datas)
{
    int HISTO_LENGTH = 100;
    std::vector<int> dataHist;  //反射强度直方图统计
    dataHist.reserve(HISTO_LENGTH);
    for(int i = 0; i < HISTO_LENGTH; i++)
        dataHist.push_back(0);

    sort(datas.begin(), datas.end());
    double min = datas.front();
    double max = datas.back();
    const double factor = HISTO_LENGTH/(max-min);

    for(unsigned int i = 0; i < datas.size(); i++) {
        double sample = datas.at(i) - min;
        int bin = round(sample*factor);    // 将sample分配到bin组
        dataHist[bin]++;
    }



    double sum = 0.0;
    for(unsigned int i = 0; i < datas.size(); i++)
        sum += datas.at(i);
    double mean =  sum / datas.size(); //均值

    double low_intensity = -1;
    double high_intensity = -1;

    bool low_found = false;
    bool high_found = false;

    std::cout << "min " << min  << ", max " << max << ", mean " << mean << std::endl;

    double bin_width = (max-min)/HISTO_LENGTH;
  //    std::cout << "bin_width: " << bin_width << std::endl;

    std::map<double,int> hist_map;
    for(unsigned int i = 0; i < dataHist.size(); i++)
        hist_map.insert(std::make_pair(dataHist.at(i), i));
    std::map<double,int>::reverse_iterator iter;
    iter = hist_map.rbegin();
    while(!low_found || !high_found)
    {
        int index = iter->second;
        double bin_edge = bin_width*double(index)+min;

        if (bin_edge > mean && !high_found)
        {
            high_found = true;
            high_intensity = bin_edge;
        }
        if (bin_edge < mean && !low_found)
        {
            low_found = true;
            low_intensity = bin_edge;
        }
        iter++;
    }


//    std::cout << low_intensity << " " <<  high_intensity << std::endl;

    /// 画出直方图统计图
  #if 0
    cv::Mat image2 = cv::Mat::zeros(600,600, CV_8UC3);
    for(int i=0;i<HISTO_LENGTH;i++){

        double height=dataHist[i];//计算高度
        //画出对应的高度图
        cv::rectangle(image2,cv::Point(i*2,600), cv::Point((i+1)*2 - 1, 600-height), CV_RGB(255,255,255));
    }
    cv::imshow("hist",image2);
    cv::waitKey(5);
  #endif

    return Eigen::Vector2d(low_intensity, high_intensity);
}

Eigen::Vector2d LidarCornersEst::get_gray_zone(myPointCloud::Ptr cloud, double rate)
{
  Eigen::Vector2d gray_zone;
  std::vector<Eigen::Vector3d> point3d;
  std::vector<double> intensitys;

  for(size_t ith = 0; ith < cloud->size(); ith++){
      Eigen::Vector3d pt;
      myPoint p = cloud->at(ith);
      pt(0) = p.x;
      pt(1) = p.y;
      pt(2) = p.z;
      point3d.push_back(pt);
      intensitys.push_back(p.intensity);
  }

  Eigen::Vector2d RLRH;
  RLRH = calHist(intensitys);

  gray_zone(0) = ((rate-1)*RLRH(0)+RLRH(1))/rate;
  gray_zone(1) = (RLRH(0)+(rate-1)*RLRH(1))/rate;

  std::cout << "rate: " << rate << ", gray_zone: " << gray_zone.transpose() << std::endl;

  return gray_zone;
}

Eigen::Matrix4f LidarCornersEst::transformbyPCA(myPointCloud::Ptr input_cloud, myPointCloud::Ptr &output_cloud)
{
    /// PCA 降维度
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*input_cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*input_cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenValuesPCA = eigen_solver.eigenvalues();

    std::cout << eigenVectorsPCA.col(1) << std::endl;

    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    std::cout << eigenVectorsPCA.col(2) << std::endl;

    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());

    pcl::PointCloud<myPoint>::Ptr PCA_cloud(new pcl::PointCloud<myPoint>);
    pcl::transformPointCloud(*input_cloud, *PCA_cloud, transform);

//    cout << "PCA eigen values \n";
    std::cout << "eigenValuesPCA: " << eigenValuesPCA.transpose()  << std::endl;
    std::cout << eigenVectorsPCA << std::endl;

    output_cloud = PCA_cloud;

    pca_matrix = transform;
    m_cloud_PCA = PCA_cloud;

    return transform;
}

void LidarCornersEst::PCA()
{
  myPointCloud::Ptr PCA_cloud(new myPointCloud);
  transformbyPCA(m_cloud_chessboard, PCA_cloud);

  m_gray_zone = get_gray_zone(m_cloud_chessboard, 3.0);
}

bool LidarCornersEst::get_corners(std::vector<Eigen::Vector3d> &corners)
{
    /// 拟合虚拟棋盘格
    bool useOutofBoard = true;

    bool topleftWhite = true;
    bool inverse_flag = true;

    myPointCloud corners_cloud;
    pcl::PointCloud<myPoint>::Ptr optim_cloud(new pcl::PointCloud<myPoint>);

    visual_corners.reset();
    Optimization optim;
    Eigen::Vector3d theta_t(0,0,0);
    while (visual_corners.m_confirm_flag == false)
    {
      if (visual_corners.update_flag){

        visual_corners.update_flag = false;

        if(visual_corners.inverse_change){
          visual_corners.inverse_change = false;
          inverse_flag = !inverse_flag;
        }
        if(visual_corners.top_white_change)
        {
          visual_corners.top_white_change = false;
          topleftWhite = !topleftWhite;

          useOutofBoard = true;
          optim.get_theta_t(m_cloud_PCA, m_grid_size, m_gray_zone,
                            topleftWhite, m_grid_length, theta_t, useOutofBoard);
          useOutofBoard = false;
          optim.get_theta_t(m_cloud_PCA, m_grid_size, m_gray_zone,
                            topleftWhite, m_grid_length, theta_t, useOutofBoard);
        }


        Eigen::Affine3f transf = pcl::getTransformation(0, theta_t(1), theta_t(2), theta_t(0), 0, 0);
        pcl::transformPointCloud(*m_cloud_PCA, *optim_cloud, transf);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_by_gray_zone(optim_cloud, m_gray_zone, rgbcloud);
        visual_corners.add_rgb_cloud(rgbcloud, "optim_cloud");


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr board_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_by_gray_zone(m_cloud_chessboard, m_gray_zone, board_cloud);
        visual_corners.add_rgb_cloud(board_cloud, "board_cloud");

        getPCDcorners(pca_matrix, transf.matrix(), corners_cloud, inverse_flag);
        visual_corners.add_color_cloud(corners_cloud.makeShared(), Eigen::Vector3i(238, 0, 255), "corners", 5);//紫色



        std::cout << "k: confirm;  d: do optim again; a: rotate board; r:reject this scan" << std::endl;
      }
        visual_corners.viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }

    visual_corners.viewer->removeAllPointClouds();

    if(visual_corners.m_reject_flag){
      std::cout << "reject this scan" << std::endl;
      return false;
    }

    m_cloud_optim = optim_cloud;
    m_cloud_corners = corners_cloud.makeShared();
    cornerCloud2vector(corners_cloud, corners);

    visual_corners.reset();
    return true;

}

void LidarCornersEst::color_by_gray_zone(myPointCloud::Ptr cloud, Eigen::Vector2d gray_zone,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgbcloud)
{
    int white = 0;
    int gray = 0;
    int black = 0;

    myPoint temp;
    pcl::PointXYZRGB rgbtemp;
    for ( unsigned int i=0; i < cloud->size(); i++ )
    {
        temp = cloud->points[i];

        if(temp.intensity < gray_zone[0]) {
            temp.intensity = 50;
            rgbtemp.r = 10;
            rgbtemp.g = 10;
            rgbtemp.b = 10;
            black++;
        }
        else if(temp.intensity > gray_zone[1]) {
            temp.intensity = 255;
            rgbtemp.r = 255;
            rgbtemp.g = 255;
            rgbtemp.b = 255;
            white++;
        }
        else {
            temp.intensity = 100;
            rgbtemp.r = 255;
            rgbtemp.g = 0;
            rgbtemp.b = 0;
            gray++;
        }

        //cloud->points[i].intensity = temp.intensity;

        rgbtemp.x = temp.x;
        rgbtemp.y = temp.y;
        rgbtemp.z = temp.z;
        rgbcloud->points.push_back(rgbtemp);
    }

    cloud->points[1].intensity = 0;

//    std::cout << "rgbcloud size: " << rgbcloud->size() << std::endl;
//    std::cout << "black: " << black << " gray: " << gray<< " white: " << white <<std::endl;
}

void LidarCornersEst::getPCDcorners(Eigen::Matrix4f transPCA, Eigen::Matrix4f transOptim, myPointCloud &corners, bool inverse)
{
  Eigen::Vector2i grid_size = m_grid_size;
  if(inverse){
    grid_size(0) = m_grid_size(1);
    grid_size(1) = m_grid_size(0);
  }
  corners.clear();
  std::vector<double> x_grid_arr;
  std::vector<double> y_grid_arr;

  double temp;
  for(int i = 1; i < grid_size(0); i++) {
      temp = (i-double(grid_size(0))/2.0) * m_grid_length;
      x_grid_arr.push_back(temp);
  }
  for(int i = 1; i < grid_size(1); i++) {
    temp = (i - double(grid_size(1))/2.0) * m_grid_length;
    y_grid_arr.push_back(temp);
  }

  //std::cout << x_grid_arr.size() << " " << y_grid_arr.size() << std::endl;

  for(unsigned int i = 0; i < x_grid_arr.size(); i++)
    for(unsigned int j = 0; j < y_grid_arr.size(); j++) {

      myPoint pt;
      pt.x = 0;
      pt.y = x_grid_arr.at(i);
      pt.z = y_grid_arr.at(j);
      pt.intensity = 50;
      corners.push_back(pt);

    }

    Eigen::Isometry3f T_optim = Eigen::Isometry3f::Identity();
    T_optim.rotate(transOptim.block<3,3>(0,0));
    T_optim.pretranslate(transOptim.block<3,1>(0,3));

    Eigen::Isometry3f T_PCA = Eigen::Isometry3f::Identity();
    T_PCA.rotate(transPCA.block<3,3>(0,0));
    T_PCA.pretranslate(transPCA.block<3,1>(0,3));

    Eigen::Isometry3f T_lidar2board = Eigen::Isometry3f::Identity();
    T_lidar2board = T_PCA.inverse() * T_optim.inverse();


    pcl::transformPointCloud(corners, corners, transOptim.inverse());
    pcl::transformPointCloud(corners, corners, transPCA.inverse());


//    cout << "transOptim: " << endl;
//    cout << transOptim << endl;
//    cout << "transPCA: " << endl;
    //    cout << transPCA << endl;
}


void LidarCornersEst::cornerCloud2vector(myPointCloud corner_cloud, std::vector<Eigen::Vector3d> &corners)
{
  Eigen::Vector3d point;
  myPoint temp;
  for(unsigned int i = 0; i < corner_cloud.size(); i++) {
      temp = corner_cloud.points.at(i);
      point(0) = temp.x;
      point(1) = temp.y;
      point(2) = temp.z;
      corners.push_back(point);
  }
}



