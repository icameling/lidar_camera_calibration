#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>                               /// std::to_string()
#include <algorithm>                            /// std::reverse

/// read rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

/// read cloud
#include <pcl_conversions/pcl_conversions.h>    /// fromROSMsg toROSMsg
#include <geometry_msgs/PointStamped.h>         /// geometry_msgs::PointStamped
#include <sensor_msgs/PointCloud2.h>

/// read image
#include <sensor_msgs/Image.h>        /// sensor_msgs::Image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>

#include "ilcc2/ImageCornersEst.h"
#include "ilcc2/LidarCornersEst.h"
#include "ilcc2/Optimization.h"

bool received_click_point = false;
myPoint click_point;



void clickedPointHandler(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    myPoint point;

    std::cout<<"received a clicked point: " << msg->point.x << "," << msg->point.y << "," << msg->point.z<<std::endl;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;

    click_point = point;

    received_click_point = true;
}


cv::Mat get_image_from_msg(const sensor_msgs::ImageConstPtr& msg_img)
{

  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return cv::Mat();
  }

  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat Image = cv_ptr->image;

  return Image;
}

void check_order(std::vector<Eigen::Vector3d>& point3d,
                 std::vector<cv::Point2f>& point2d,
                 cv::Size boardSize)
{

  std::cout << point3d.size() << std::endl;

  if(point2d.at(0).x > point2d.at(1).x)
    std::reverse(point2d.begin(), point2d.end());
  if(point3d.at(0)[2] < point3d.at(boardSize.width+1)[2])
    std::reverse(point3d.begin(), point3d.end());

  if(point3d.at(0)[1] < point3d.at(1)[1])
  {
    for(int h = 0; h < boardSize.height; h++)
    {
      int front = boardSize.width*h;
      for(int w = 0; w < boardSize.width/2; w++)
      {
          Eigen::Vector3d p = point3d.at(front+w);
          point3d.at(front+w) = point3d.at(front+boardSize.width-1-w);
          point3d.at(front+boardSize.width-1-w) = p;
      }
    }
  }


}

void show_pcd_corners(std::vector<Eigen::Vector3d> point3d,
                      std::vector<cv::Point2f> point2d) {

    cv::Mat img_show(1500, 1900, CV_8UC3, cv::Scalar::all(0));
    //cv::rectangle(img_show,cv::Point(0,0), cv::Point(img_show.cols, img_show.rows), CV_RGB(255,255,255));

    for (uint i = 0; i < point3d.size(); i++)
    {
        cv::Point2f p;
        p.x = -point3d.at(i)[1];
        p.y = -point3d.at(i)[2];
        //x=-y
        //y=-z
        //z=x

        p.x = (p.x + 0.9)*1000;
        p.y = (p.y + 0.2)*1000;
        cv::circle(img_show, p, 2, cv::Scalar(0,0,255), 3);


        std::string str = std::to_string(i);
        cv::putText(img_show, str, p, 0.8, 1.0, cv::Scalar(255,0,0),1);

        /// 绿色为像素点
        cv::circle(img_show, cv::Point2f(point2d.at(i).x*2, point2d.at(i).y*2), 2, cv::Scalar(0,0,255), 3);
        cv::putText(img_show, str, cv::Point2f(point2d.at(i).x*2, point2d.at(i).y*2), 0.8, 1.0, cv::Scalar(0,255,0),1);

    }

    cv::resize(img_show, img_show, cv::Size(img_show.cols*0.7, img_show.rows*0.7));
    cv::imshow("lidar corners", img_show);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void extrinsic2txt(std::string savefile, Eigen::Matrix4d lidar2cam)
{
  std::ofstream outfile(savefile.c_str(), std::ios_base::binary);
  outfile.write((const char*)&lidar2cam, sizeof(Eigen::Matrix4d));
  outfile.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_corners");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("ilcc2");
    ROS_INFO("ilcc2 package at %s", package_path.c_str());

    int bag_num;
    std::string bag_path_prefix, lidar_topic, image_topic, yaml_path, pose_file;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("bag_path_prefix", bag_path_prefix, "20181101_");
    nh_private.param<int>("bag_num", bag_num, 1);
    nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
    nh_private.param<std::string>("yaml_path", yaml_path, "30w.yaml");
    nh_private.param<std::string>("image_topic", image_topic, "/front");
    nh_private.param<std::string>("pose_file", pose_file, "front.bin");

    std::cout << "bag_num: " << bag_num << std::endl;
    std::cout << package_path << "/config/" << yaml_path << std::endl;

    ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);
    image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );

    LidarCornersEst::Ptr lidar_corners_est(new LidarCornersEst);
    lidar_corners_est->set_chessboard_param(package_path+"/config/"+yaml_path);


    ros::Subscriber ClickedPointSub = nh.subscribe<geometry_msgs::PointStamped> ("/clicked_point", 100, clickedPointHandler);
    ros::Publisher pubLasercloud;
    pubLasercloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);

    std::vector< std::vector<Eigen::Vector3d> >lidar_corners;
    std::vector< std::vector<cv::Point2f> > image_corners;
    std::vector<cv::Mat> m_images;

    for(int bag_idx = 1; bag_idx <= bag_num; bag_idx++)
    {
        std::cout << bag_idx << "============================================================\n";
        std::string bag_path = bag_path_prefix+std::to_string(bag_idx)+".bag";
        ROS_INFO("bag_path: %s", bag_path.c_str());

        rosbag::Bag bag_read;
        bag_read.open(bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(lidar_topic);
        topics.push_back(image_topic);

        rosbag::View view(bag_read, rosbag::TopicQuery(topics));
        std::cout << "view: " << view.size() << std::endl;

        sensor_msgs::PointCloud2ConstPtr msg_cloud_last = NULL;
        sensor_msgs::ImageConstPtr msg_image_last = NULL;
        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2ConstPtr msg_cloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg_cloud != NULL){
              msg_cloud_last = msg_cloud;
            }
            sensor_msgs::ImageConstPtr msg_image = m.instantiate<sensor_msgs::Image>();
            if (msg_image != NULL){
              msg_image_last = msg_image;
            }

            if(msg_cloud_last != NULL && msg_image_last != NULL)
              break;
        }
        bag_read.close();

        if(msg_cloud_last == NULL || msg_image_last == NULL)
        {
          ROS_WARN("can't read image topic or lidar topic");
          continue;
        }

        myPointCloud pointcloud;
        pcl::fromROSMsg(*msg_cloud_last, pointcloud);

        sensor_msgs::PointCloud2 Ponitfilted2;
        pcl::toROSMsg(pointcloud, Ponitfilted2);
        Ponitfilted2.header.stamp = Ponitfilted2.header.stamp;
        Ponitfilted2.header.frame_id = "/velodyne";
        pubLasercloud.publish(Ponitfilted2);


        ROS_INFO_STREAM("please public topic /click_point.....");
        ros::Rate loop_rate(10);

        std::vector<Eigen::Vector3d> lidar_corner;
        received_click_point = false;
        while (ros::ok())
        {
          if(received_click_point)
          {
              cv::Mat image = get_image_from_msg(msg_image_last);
              image_corners_est->findCorners(image);

              lidar_corners_est->setROI(pointcloud.makeShared(), click_point);
              if( lidar_corners_est->EuclideanCluster() )
              {
                  lidar_corner.clear();
                  ROS_WARN("get_corners");
                  if(lidar_corners_est->get_corners(lidar_corner))
                  {
                      ROS_WARN("add_corner");
                      lidar_corners.push_back(lidar_corner);
                      image_corners.push_back(image_corners_est->corners_now);
                      cv::Mat image;
                      image_corners_est->image_now.copyTo(image);
                      m_images.push_back( image );
                  }
                  break;
              }
              received_click_point = false;
          }

          pubLasercloud.publish(Ponitfilted2);
          ros::spinOnce();
          loop_rate.sleep();
        }
    }


    std::cout << lidar_corners.size() << std::endl;
    std::cout << image_corners.size() << std::endl;
    std::cout << m_images.size() << std::endl;

    /// data ready
    std::vector<Eigen::Vector3d> point3d_all;
    std::vector<cv::Point2f> point2d_all;
    for(unsigned int i = 0 ; i < lidar_corners.size(); i++){
      check_order(lidar_corners.at(i), image_corners.at(i), image_corners_est->m_board_size);
      show_pcd_corners(lidar_corners.at(i), image_corners.at(i));


      point3d_all.insert(point3d_all.end(), lidar_corners.at(i).begin(), lidar_corners.at(i).end());
      point2d_all.insert(point2d_all.end(), image_corners.at(i).begin(), image_corners.at(i).end());
    }


//    Eigen::Vector3d r_ceres(0,0,0);
//    Eigen::Vector3d t_ceres(-0.23,0,0);
//    Optimization optim;
//    Eigen::Isometry3d T = optim.solvePose3d2dError(point3d_all, point2d_all, image_corners_est->camK, r_ceres, t_ceres);
//    extrinsic2txt(package_path+"/config/"+pose_file, T.matrix());


//    Eigen::Matrix3d R = T.rotation();
//    Eigen::Vector3d t = T.translation();
//    image_corners_est->setRt(R, t);

//    for(unsigned int i = 0 ; i < lidar_corners.size(); i++){
//      cv::Mat image;
//      cv::cvtColor(m_images.at(i), image, CV_GRAY2RGB);
//      image_corners_est->show_calib_result(lidar_corners.at(i), image_corners.at(i), image);
//      cv::imshow(std::to_string(i), image);
//      cv::waitKey(0);
//    }


    return 0;
}

