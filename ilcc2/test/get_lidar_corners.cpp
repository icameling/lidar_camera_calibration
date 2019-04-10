#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

// read rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pcl_conversions/pcl_conversions.h>    /// fromROSMsg toROSMsg
#include <geometry_msgs/PointStamped.h>         /// geometry_msgs::PointStamped
#include <sensor_msgs/PointCloud2.h>

#include "ilcc2/LidarCornersEst.h"

ros::Publisher pubLaserCloud;
ros::Publisher pubLaserChessBoard;
ros::Publisher pubLaserPCA;
ros::Publisher pubLaserOptim;
ros::Publisher pubLaserCorners;


bool received_click_point = false;
myPoint click_point;

void save_corners2txt(myPointCloud::Ptr cloud, std::string filename) {
  std::ofstream outfile(filename.c_str(), std::ios_base::trunc);

  myPoint temp;
  for(unsigned int i = 0; i < cloud->size(); i++) {
      temp = cloud->points[i];
      outfile << temp.x << " " << temp.y << " " << temp.z  << endl;
  }
  outfile.close();
}

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

void publish_all_cloud(myPointCloud::Ptr cloud_chessBoard,
                       myPointCloud::Ptr cloud_optim,
                       myPointCloud::Ptr cloud_corners){

  sensor_msgs::PointCloud2 ChessBoard_msg;
  pcl::toROSMsg(*cloud_chessBoard, ChessBoard_msg);
  ChessBoard_msg.header.stamp = ros::Time::now();
  ChessBoard_msg.header.frame_id = "/velodyne";
  pubLaserChessBoard.publish(ChessBoard_msg);

  sensor_msgs::PointCloud2 optim_msg;
  pcl::toROSMsg(*cloud_optim, optim_msg);
  optim_msg.header.stamp = ros::Time::now();
  optim_msg.header.frame_id = "/velodyne";
  pubLaserOptim.publish(optim_msg);

  sensor_msgs::PointCloud2 corner_msg;
  pcl::toROSMsg(*cloud_corners, corner_msg);
  corner_msg.header.stamp = ros::Time::now();
  corner_msg.header.frame_id = "/velodyne";
  pubLaserCorners.publish(corner_msg);

}

void publish_cloud(ros::Publisher &pub, myPointCloud::Ptr cloud_to_pub ){

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_to_pub, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "/velodyne";
  pub.publish(cloud_msg);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_corners");
    ros::NodeHandle nh;


    std::string package_path = ros::package::getPath("ilcc2");
    ROS_INFO("ilcc2 package at %s", package_path.c_str());

    int bag_num;
    std::string bag_path_prefix, lidar_topic, yaml_path, camera_name;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("bag_path_prefix", bag_path_prefix, "20181101_");
    nh_private.param<int>("bag_num", bag_num, 1);
    nh_private.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");

    nh_private.param<std::string>("camera_name", camera_name, "front");
    nh_private.param<std::string>("yaml_path", yaml_path, "front.yaml");


    std::cout << "bag_num: " << bag_num << std::endl;
    std::cout << package_path << "/config/" << yaml_path << std::endl;

    LidarCornersEst::Ptr lidar_corners_est(new LidarCornersEst);
    lidar_corners_est->register_viewer();
    lidar_corners_est->set_chessboard_param(package_path+"/config/"+yaml_path);


    ros::Subscriber ClickedPointSub = nh.subscribe<geometry_msgs::PointStamped> ("/clicked_point", 100, clickedPointHandler);


    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    pubLaserChessBoard = nh.advertise<sensor_msgs::PointCloud2>("/ChessBoard", 10);
    pubLaserPCA = nh.advertise<sensor_msgs::PointCloud2>("/pca_cloud", 10);
    pubLaserOptim = nh.advertise<sensor_msgs::PointCloud2>("/Optim_cloud", 10);
    pubLaserCorners = nh.advertise<sensor_msgs::PointCloud2>("/lidar_corners", 10);



    std::vector< std::vector<Eigen::Vector3d> >lidar_corners;

    for(int bag_idx = 1; bag_idx <= bag_num; bag_idx++)
    {
        std::cout << bag_idx << "============================================================\n";
        std::string bag_path = bag_path_prefix+std::to_string(bag_idx)+".bag";
        ROS_INFO("bag_path: %s", bag_path.c_str());

        rosbag::Bag bag_read;
        bag_read.open(bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(lidar_topic);

        rosbag::View view(bag_read, rosbag::TopicQuery(topics));
        std::cout << "view: " << view.size() << std::endl;

        sensor_msgs::PointCloud2ConstPtr msg_cloud_last = NULL;
        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2ConstPtr msg_cloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg_cloud != NULL){
              msg_cloud_last = msg_cloud;
            }

            if(msg_cloud_last != NULL)
              break;
        }
        bag_read.close();

        if(msg_cloud_last == NULL)
        {
          ROS_WARN("can't read lidar topic");
          continue;
        }

        myPointCloud pointcloud;
        pcl::fromROSMsg(*msg_cloud_last, pointcloud);

        sensor_msgs::PointCloud2 Ponitfilted2;
        pcl::toROSMsg(pointcloud, Ponitfilted2);
        Ponitfilted2.header.stamp = Ponitfilted2.header.stamp;
        Ponitfilted2.header.frame_id = "/velodyne";
        pubLaserCloud.publish(Ponitfilted2);


        ROS_INFO_STREAM("please public topic /click_point.....");
        ros::Rate loop_rate(10);

        std::vector<Eigen::Vector3d> lidar_corner;
        received_click_point = false;
        while (ros::ok())
        {
          if(received_click_point)
          {
              received_click_point = false;
              lidar_corners_est->setROI(pointcloud.makeShared(), click_point);

              //lidar_corners_est->min_cut_segmentation();
//              lidar_corners_est->EuclideanCluster();
//              break;
              if( lidar_corners_est->EuclideanCluster() )
              {
                  lidar_corner.clear();
                  lidar_corners_est->PCA();
                  publish_cloud(pubLaserChessBoard, lidar_corners_est->m_cloud_chessboard);
                  publish_cloud(pubLaserPCA, lidar_corners_est->m_cloud_PCA);
                  if(lidar_corners_est->get_corners(lidar_corner))
                  {
                      ROS_WARN("add_corner");
                      std::string savepath = package_path+"/process_data/"+camera_name+"_lidar_"+std::to_string(bag_idx)+".txt";
                      save_corners2txt(lidar_corners_est->m_cloud_corners, savepath);

                      publish_cloud(pubLaserOptim, lidar_corners_est->m_cloud_optim);
                      publish_cloud(pubLaserCorners, lidar_corners_est->m_cloud_corners);
                  }
                  break;
              }
          }

          pubLaserCloud.publish(Ponitfilted2);
          ros::spinOnce();
          loop_rate.sleep();
        }
    }



    return 0;
}

