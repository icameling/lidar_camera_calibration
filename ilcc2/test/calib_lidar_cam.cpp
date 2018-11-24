#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>                               /// std::to_string()
#include <algorithm>                            /// std::reverse

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ilcc2/ImageCornersEst.h"
#include "ilcc2/LidarCornersEst.h"
#include "ilcc2/Optimization.h"


void show_pcd_corners(std::vector<Eigen::Vector3d> point3d,
                      std::vector<cv::Point2d> point2d) {

    cv::Mat img_show(2200, 2000, CV_8UC3, cv::Scalar::all(0));
    //cv::rectangle(img_show,cv::Point(0,0), cv::Point(img_show.cols, img_show.rows), CV_RGB(255,255,255));

    for (uint i = 0; i < point3d.size(); i++)
    {
        cv::Point2d p;

        p.x = point3d.at(i)[0];
        p.y = point3d.at(i)[1];

        p.x = (p.x + 0.9)*1000;
        p.y = (p.y + 0.2)*1000;
        cv::circle(img_show, p, 2, cv::Scalar(255,0,0), 3);


        std::string str = std::to_string(i);
        cv::putText(img_show, str, p, 0.8, 1.0, cv::Scalar(0,0,255),1);

        /// 绿色为像素点
        cv::circle(img_show, cv::Point2d(point2d.at(i).x*2, point2d.at(i).y*2), 2, cv::Scalar(255,0,0), 3);
        cv::putText(img_show, str, cv::Point2d(point2d.at(i).x*2, point2d.at(i).y*2), 0.8, 1.0, cv::Scalar(0,255,0),1);

    }

    cv::resize(img_show, img_show, cv::Size(img_show.cols*0.5, img_show.rows*0.5));
    cv::imshow("lidar corners", img_show);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


 /// 这个是为了让相机和激光坐标系方向一致
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_corners");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("ilcc2");
    ROS_INFO("ilcc2 package at %s", package_path.c_str());

    int bag_num;
    std::string yaml_path, pose_file, camera_name;
    ros::NodeHandle nh_private("~");
    nh_private.param<int>("bag_num", bag_num, 1);
    nh_private.param<std::string>("yaml_path", yaml_path, "front.yaml");
    nh_private.param<std::string>("camera_name", camera_name, "front");
    nh_private.param<std::string>("pose_file", pose_file, "front.bin");

    std::cout << "bag_num: " << bag_num << std::endl;
    std::cout << package_path << "/config/" << yaml_path << std::endl;

    ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);
    image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );



    std::vector<std::vector<Eigen::Vector3d> > lidar_corners_piece;
    std::vector<std::vector<cv::Point2d> > image_corners_piece;
    std::vector<cv::Mat> m_images;

    int corner_num = image_corners_est->m_board_size.width * image_corners_est->m_board_size.height;
    Eigen::Isometry3d T_lidar2cam_axis_roughly = get_lidar2cam_axis_roughly(camera_name);

    for(int bag_idx = 1; bag_idx <= bag_num; bag_idx++)
    {

        std::string lidar_corner_path = package_path+"/process_data/"+camera_name+"_lidar_"+std::to_string(bag_idx)+".txt";
        std::string cam_corner_path = package_path+"/process_data/"+camera_name+std::to_string(bag_idx)+".txt";
        std::string image_path = package_path+"/process_data/"+camera_name+std::to_string(bag_idx)+".jpg";

        std::cout << lidar_corner_path << std::endl;
        std::cout << cam_corner_path << std::endl;

        std::vector<Eigen::Vector3d> point3d;
        std::vector<cv::Point2d> point2d;

        image_corners_est->read_cam_corners(cam_corner_path, corner_num, point2d);
        image_corners_est->read_lidar_corners(lidar_corner_path, corner_num, point3d);

        for(unsigned int i = 0 ; i < point3d.size(); i++)  /// 这个是为了让相机和激光坐标系方向一致
            point3d.at(i) = T_lidar2cam_axis_roughly * point3d.at(i);

        image_corners_est->check_order_lidar(point3d, image_corners_est->m_board_size);
        image_corners_est->check_order_cam(point2d, image_corners_est->m_board_size);

        lidar_corners_piece.push_back(point3d);
        image_corners_piece.push_back(point2d);

        cv::Mat image = cv::imread(image_path);
        m_images.push_back( image );
    }


    for(unsigned int i = 0 ; i < lidar_corners_piece.size(); i++){
      show_pcd_corners(lidar_corners_piece.at(i), image_corners_piece.at(i));
    }


    /// data ready
    std::vector<Eigen::Vector3d> lidar_corners;
    std::vector<cv::Point2d> image_corners;

    for(unsigned int i = 0 ; i < m_images.size(); i++)
    {
      lidar_corners.insert(lidar_corners.end(), lidar_corners_piece.at(i).begin(), lidar_corners_piece.at(i).end());
      image_corners.insert(image_corners.end(), image_corners_piece.at(i).begin(), image_corners_piece.at(i).end());
    }

    std::cout << lidar_corners.size() << std::endl;
    std::cout << image_corners.size() << std::endl;
    std::cout << m_images.size() << std::endl;

    Eigen::Vector3d r_ceres(0,0,0);
    Eigen::Vector3d t_ceres(0,0,0);
    Optimization optim;
    Eigen::Isometry3d T_lidar2cam = optim.solvePose3d2dError(lidar_corners, image_corners, image_corners_est->camK, r_ceres, t_ceres);

    Eigen::Matrix3d R = T_lidar2cam.rotation();
    Eigen::Vector3d t = T_lidar2cam.translation();
    image_corners_est->setRt(R, t);


    T_lidar2cam = T_lidar2cam * T_lidar2cam_axis_roughly;
    image_corners_est->extrinsic2txt(package_path+"/config/"+pose_file, T_lidar2cam.matrix());


    for(unsigned int i = 0 ; i < m_images.size(); i++){
      cv::Mat image = m_images.at(i);
      //cv::cvtColor(m_images.at(i), image, CV_GRAY2RGB);
      image_corners_est->show_calib_result(lidar_corners_piece.at(i), image_corners_piece.at(i), image);
      cv::imshow(std::to_string(i), image);
      cv::waitKey(0);
    }


    return 0;
}

