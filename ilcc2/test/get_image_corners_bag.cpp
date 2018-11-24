#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

/// read rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/Image.h>        /// sensor_msgs::Image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>


#include "ilcc2/ImageCornersEst.h"

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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_corners");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("ilcc2");
    ROS_INFO("ilcc2 package at %s", package_path.c_str());

    int bag_num;
    bool save_image_flag;
    std::string bag_path_prefix, image_topic, yaml_path, camera_name;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("bag_path_prefix", bag_path_prefix, "20181101_");
    nh_private.param<int>("bag_num", bag_num, 1);
    nh_private.param<std::string>("yaml_path", yaml_path, "30w.yaml");
    nh_private.param<std::string>("image_topic", image_topic, "/front");
    nh_private.param<std::string>("camera_name", camera_name, "front");
    nh_private.param<bool>("save_image_flag", save_image_flag, "true");

    std::cout << "bag_num: " << bag_num << std::endl;
    std::cout << package_path << "/config/" << yaml_path << std::endl;

    ImageCornersEst::Ptr image_corners_est(new ImageCornersEst);
    image_corners_est = ImageCornersEst::Ptr(new ImageCornersEst(package_path+"/config/"+yaml_path, true) );


    for(int bag_idx = 1; bag_idx <= bag_num; bag_idx++)
    {
        std::cout << bag_idx << "============================================================\n";
        std::string bag_path = bag_path_prefix+std::to_string(bag_idx)+".bag";
        ROS_INFO("bag_path: %s", bag_path.c_str());

        rosbag::Bag bag_read;
        bag_read.open(bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(image_topic);

        rosbag::View view(bag_read, rosbag::TopicQuery(topics));
        std::cout << "view: " << view.size() << std::endl;

        sensor_msgs::ImageConstPtr msg_image_last = NULL;
        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::ImageConstPtr msg_image = m.instantiate<sensor_msgs::Image>();
            if (msg_image != NULL){
              msg_image_last = msg_image;
            }

            if(msg_image_last != NULL)
              break;
        }
        bag_read.close();

        if(msg_image_last == NULL)
        {
          ROS_WARN("can't read image topic");
          continue;
        }


        ROS_INFO("findCorners");
        cv::Mat image = get_image_from_msg(msg_image_last);

        cv::Mat rectifyImage;
        image_corners_est->undistort_image(image, rectifyImage);

        //image_corners_est->findCorners(image);

        if(save_image_flag)
          cv::imwrite(package_path+"/process_data/"+camera_name+std::to_string(bag_idx)+".jpg", rectifyImage);

    }


    return 0;
}

