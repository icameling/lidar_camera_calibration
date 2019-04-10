#include "ilcc2/ImageCornersEst.h"
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>    /// cv::findChessboardCorners
#include <opencv2/imgproc.hpp>    /// cv::undistort

#include "camodocal/chessboard/Chessboard.h"


bool ImageCornersEst::getRectifyParam(std::string cam_yaml, bool stereo)
{
  cv::FileStorage fs;
  fs.open(cam_yaml, cv::FileStorage::READ);

  if ( !fs.isOpened() )
  {
      std::cerr << "can not open " << cam_yaml << std::endl;
      return false;
  }
  fs["K"] >> camK;
  fs["d"] >> distort_param;
  m_image_size.width  = static_cast<int>(fs["Camera.width"]);
  m_image_size.height = static_cast<int>(fs["Camera.height"]);
  m_grid_length = static_cast<double>(fs["grid_length"]);
  m_board_size.width = static_cast<int>(fs["corner_in_x"]);
  m_board_size.height = static_cast<int>(fs["corner_in_y"]);

  if(stereo) {
    fs["Krr"]>>Kr;
    fs["drr"]>>dr;
    fs["R"]>>car_R;
    fs["t"]>>car_t;
  }

  fs.release();

  m_fx = camK.at<double>(0,0);
  m_cx = camK.at<double>(0,2);
  m_fy = camK.at<double>(1,1);
  m_cy = camK.at<double>(1,2);

  std::cout << "camK: \n"  << camK << std::endl;
  std::cout << "dist: \n"  << distort_param << std::endl;
  std::cout << "grid_length: " << m_grid_length << std::endl;
  std::cout << "corner_in_x: " << m_board_size.width << std::endl;
  std::cout << "corner_in_y: " << m_board_size.height << std::endl;

  if(stereo){
    std::cout << "Kr: \n"  << Kr << std::endl;
    std::cout << "dr: \n"  << dr << std::endl;
    std::cout << "car_R: \n"  << car_R << std::endl;
    std::cout << "car_t: \n"  << car_t << std::endl;
  }

  return true;
}

void ImageCornersEst::undistort_image(cv::Mat image, cv::Mat &rectify_image)
{
  cv::undistort(image, rectify_image, camK, distort_param, camK);
}

void ImageCornersEst::undistort_stereo_image(cv::Mat image, cv::Mat &rectify_image)
{
  int downsize=1;
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(camK,distort_param,Kr,dr,m_image_size,car_R,car_t,
                    R1,R2,P1,P2,Q,cv::CALIB_ZERO_DISPARITY,0,
                    cv::Size(m_image_size.width/downsize,m_image_size.height/downsize));
  cv::Mat M1l,M2l;      // can be saved to save time
  cv::initUndistortRectifyMap(camK,distort_param,R1,P1.rowRange(0,3).colRange(0,3),
                              cv::Size(m_image_size.width/downsize,m_image_size.height/downsize),CV_32F,M1l,M2l);

  cv::remap(image,rectify_image,M1l,M2l,cv::INTER_LINEAR);
}


bool ImageCornersEst::findCorners(cv::Mat image, bool undist)
{
  bool useOpenCV = false;

  cv::Mat rectifyImage;
  if(undist)
    undistort_image(image, rectifyImage);
  else
    image.copyTo(rectifyImage);

  camodocal::Chessboard chessboard(m_board_size, rectifyImage);
  chessboard.findCorners(useOpenCV);
  std::vector<cv::Point2f> corners = chessboard.getCorners();
  if (chessboard.cornersFound())
  {
    if(useOpenCV)
      cv::cornerSubPix(rectifyImage, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    corners_now = corners;
  }
  else
      std::cerr << "can't Detected chessboard.......................................\n";

  rectifyImage.copyTo(image_now);

  cv::Mat image_show;
  if(rectifyImage.channels() == 1)
    cv::cvtColor(rectifyImage, image_show, CV_GRAY2RGB);
  else
    rectifyImage.copyTo(image_show);
  /// 画图哈
  if(chessboard.cornersFound())
  {
    cv::drawChessboardCorners(image_show, m_board_size, corners, chessboard.cornersFound());
    cv::putText(image_show, "0", corners.at(0), 1, 1, cv::Scalar(255,0,255),2);
  }
  else
    cv::putText(image_show, "can't Detected chessboard", cv::Point(10, image_show.rows - 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 0.5, CV_AA);

  image_chessboard = image_show;


//  cv::resize(image_show, image_show, cv::Size(image_show.cols/1.0, image_show.rows/1.0));
  cv::imshow("Corners", image_show);
  cv::waitKey(5);

  return chessboard.cornersFound();
}


bool ImageCornersEst::spaceToPlane(Eigen::Vector3d P_w, Eigen::Vector2d &P_cam, double dis)
{
        Eigen::Vector3d P_c = m_R * P_w + m_t;
//        Eigen::Vector3d P_c = P_w;

        if ( P_c[2]<0 || P_c[2] > dis) {
            return false;
        }

        // Transform to model plane
        double u = P_c[0] / P_c[2];
        double v = P_c[1] / P_c[2];

        P_cam(0) = m_fx * u + m_cx;
        P_cam(1) = m_fy * v + m_cy;

        if ( P_cam(0)>0 && P_cam(0)<m_image_size.width && P_cam(1)>0 && P_cam(1)<m_image_size.height)
          return true;
        else
          return false;
}

void ImageCornersEst::show_calib_result(std::vector<Eigen::Vector3d> point3d, std::vector<cv::Point2d> point2d, cv::Mat &image)
{
    float errorSum = 0.0f;
    float errorMax = 0;

    for(unsigned int i = 0; i < point3d.size(); i++)
    {
      Eigen::Vector3d Pw = point3d.at(i);
      Eigen::Vector2d Pcam;
      if ( spaceToPlane(Pw, Pcam) )
      {

          cv::Point2d pEst(Pcam[0], Pcam[1]);
          cv::Point2d pObs = point2d.at(i);

          cv::circle(image, pEst, 1, cv::Scalar(0,0,255), 2);//r
          cv::circle(image, pObs, 1, cv::Scalar(255,0,0), 2);//b

          float error = cv::norm(pObs - pEst);

  //              cout << counter << " " << error << " " << pEst.x << " " << pEst.y << " "
  //                   << pObs.x << " " << pObs.y << " " <<endl;

          errorSum += error;
          if (error > errorMax)
          {
              errorMax = error;
          }
      }
    }

    std::ostringstream oss;
    oss << "Reprojection error: avg = " << errorSum / 35.0 << "   max = " << errorMax;

    cv::putText(image, oss.str(), cv::Point(10, image.rows - 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                0.5, CV_AA);
    std::cout << "  " << oss.str() << std::endl;
}

void ImageCornersEst::split(std::string &s, std::string &delim, std::vector<std::string> &ret)
{
  size_t last = 0;
  size_t index = s.find_first_of(delim, last);
  while (index != std::string::npos)
  {
      ret.push_back(s.substr(last, index - last));
      last = index + 1;
      index = s.find_first_of(delim, last);
  }
  if (index - last > 0)
  {
      ret.push_back(s.substr(last, index - last));
  }
}

void ImageCornersEst::read_cam_corners(std::string filename, int num, std::vector<cv::Point2d> &point2d)
{
  std::vector<std::vector<cv::Point2d> > Corners;

  std::ifstream infile;
  infile.open(filename);

  std::string s;
  int counter = 0;
  std::string dlim = " ";
  while ( std::getline(infile, s))
  {
      std::vector<std::string> slist;
      split(s, dlim, slist);
      std::vector<cv::Point2d> corner;
      for (uint i = 0; i < slist.size(); i++)
      {
          std::stringstream ss;
          ss << slist[i];
          cv::Point2d p;
          ss >> p.x;
          p.y = 0;
          corner.push_back(p);
          counter++;
      }
      Corners.push_back(corner);

      if(counter >= num)
        break;

  }

  counter = 0;
  while ( std::getline(infile, s))
  {
      std::vector<std::string> slist;
      split(s, dlim, slist);
      for (uint i = 0; i < slist.size(); i++)
      {
          std::stringstream ss;
          ss << slist[i];
          ss >> Corners[counter][i].y;
      }
      counter++;
  }

  infile.close();


  /// ok
  if( Corners.size() != m_board_size.height){
    for(unsigned int w = 0; w < Corners.at(0).size(); w++)
      for(unsigned int h = 0; h < Corners.size(); h++)
        point2d.push_back(Corners[h][w]);

  }
  else
  {
    for(unsigned int h = 0; h < Corners.size(); h++)
      for(unsigned int w = 0; w < Corners.at(0).size(); w++)
        point2d.push_back(Corners[h][w]);

  }


  //std::cout << " camera corner size: " << point2d.size() << std::endl;
}

void ImageCornersEst::read_lidar_corners(std::string filename, int num, std::vector<Eigen::Vector3d> &point3d)
{

  std::ifstream infile;

  int counter = 0;
  infile.open(filename.c_str());
  while ( !infile.eof() && counter < num)
  {
      float x, y, z;
      infile>>x>>y>>z;
      point3d.push_back( Eigen::Vector3d(x,y,z));

      counter++;
  }
  infile.close();

  //std::cout << " lidar  corner size: " << point3d.size() << std::endl;
}

void ImageCornersEst::extrinsic2txt(std::string savefile, Eigen::Matrix4d lidar2cam)
{
  std::ofstream outfile(savefile.c_str(), std::ios_base::binary);
  outfile.write((const char*)&lidar2cam, sizeof(Eigen::Matrix4d));
  outfile.close();
}
//void ImageCornersEst::txt2extrinsic(std::string filepath)
//{
//    Eigen::Matrix4d lidar2imu, lidar2cam, imu2cam;
////    imu2cam << -0.99927393, -0.03500139, -0.01505043, 0.12083473,
////                0.01431119, 0.02127604, -0.99967121, -0.48445726,
////                 0.03531009, -0.99916076, -0.02075968, 0.04976994,
////                 0.,          0.,          0. ,         1.;

////    lidar2imu <<    0.99773566 ,-0.0304899, 0.05994924, 0.06959133,
////                     0.03146003 ,0.99938782 ,-0.01530552, 0.07725167,
////                   -0.05944587, 0.01715687, 0.99808408, -0.51335252,
////                  0.,          0.,          0. ,         1.;

//    imu2cam <<-0.99915274 ,-0.03940741 ,-0.01186795, 0.10650074,
//              0.01202265 ,-0.00369103, -0.99992091, -0.06539255,
//              0.03936049, -0.99921641 ,0.00416168 ,0.01751488,
//                 0.,          0.,          0. ,         1.;

//    lidar2imu <<0.99680228, -0.06535214, 0.04598159, 0.08182204,
//                  0.06442578, 0.9976941, 0.02134946, 0.07283407,
//                 -0.04727079 ,-0.01831879 ,0.99871412, -0.11007261,
//                  0.,          0.,          0. ,         1.;

//    Eigen::Isometry3d T_imu2cam = Eigen::Isometry3d::Identity();
//    T_imu2cam.rotate(imu2cam.block<3,3>(0,0));
//    T_imu2cam.pretranslate(imu2cam.block<3,1>(0,3));

//    Eigen::Isometry3d T_lidar2imu = Eigen::Isometry3d::Identity();
//    T_lidar2imu.rotate(lidar2imu.block<3,3>(0,0));
//    T_lidar2imu.pretranslate(lidar2imu.block<3,1>(0,3));


//    Eigen::Isometry3d T_lidar2cam_temp = Eigen::Isometry3d::Identity();
//    T_lidar2cam_temp =  T_imu2cam * T_lidar2imu;
////    T_lidar2cam_temp.rotate(lidar2cam.block<3,3>(0,0));
////    T_lidar2cam_temp.pretranslate(lidar2cam.block<3,1>(0,3));

//    m_R = T_lidar2cam_temp.rotation();
//    m_t = T_lidar2cam_temp.translation();

//    T_lidar2cam = T_lidar2cam_temp;

//    std::cout << "cam2lidar: \n" << T_lidar2cam_temp.inverse().matrix() << std::endl;

//}
void ImageCornersEst::txt2extrinsic(std::string filepath)
{
    Eigen::Matrix4d lidar2cam;
    std::ifstream infile(filepath, std::ios_base::binary);
    infile.read((char*)&lidar2cam, sizeof(Eigen::Matrix4d));
    infile.close();
    std::cout << "lidar2cam\n" << lidar2cam << std::endl;

    Eigen::Isometry3d T_lidar2cam_temp = Eigen::Isometry3d::Identity();
    T_lidar2cam_temp.rotate(lidar2cam.block<3,3>(0,0));
    T_lidar2cam_temp.pretranslate(lidar2cam.block<3,1>(0,3));

    m_R = lidar2cam.block<3,3>(0,0);
    m_t = lidar2cam.block<3,1>(0,3);

    T_lidar2cam = T_lidar2cam_temp;

    std::cout << "cam2lidar: \n" << T_lidar2cam_temp.inverse().matrix() << std::endl;

}

void ImageCornersEst::HSVtoRGB(int h, int s, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
  // convert from HSV/HSB to RGB color
  // R,G,B from 0-255, H from 0-260, S,V from 0-100
  // ref http://colorizer.org/

  // The hue (H) of a color refers to which pure color it resembles
  // The saturation (S) of a color describes how white the color is
  // The value (V) of a color, also called its lightness, describes how dark the color is

  int i;


  float RGB_min, RGB_max;
  RGB_max = v*2.55f;
  RGB_min = RGB_max*(100 - s)/ 100.0f;

  i = h / 60;
  int difs = h % 60; // factorial part of h

  // RGB adjustment amount by hue
  float RGB_Adj = (RGB_max - RGB_min)*difs / 60.0f;

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
  default:		// case 5:
    *r = RGB_max;
    *g = RGB_min;
    *b = RGB_max - RGB_Adj;
    break;
  }
}

void ImageCornersEst::check_order_cam(std::vector<cv::Point2d> &point2d, cv::Size boardSize)
{
  /// check point2D
  if(point2d.at(0).y > point2d.at(boardSize.width+1).y){
    for(int h = 0; h < boardSize.height/2; h++)
    {
      int front = boardSize.width*h;
      int end = boardSize.width*(boardSize.height-1-h);
      for(int w = 0; w < boardSize.width; w++)
      {
          cv::Point2d p = point2d.at(front+w);
          point2d.at(front+w) = point2d.at(end+w);
          point2d.at(end+w) = p;
      }
    }
  }
  if(point2d.at(0).x > point2d.at(1).x){
    for(int h = 0; h < boardSize.height; h++)
    {
      int front = boardSize.width*h;
      for(int w = 0; w < boardSize.width/2; w++)
      {
          cv::Point2d p = point2d.at(front+w);
          point2d.at(front+w) = point2d.at(front+boardSize.width-1-w);
          point2d.at(front+boardSize.width-1-w) = p;
      }
    }
  }

}

void ImageCornersEst::check_order_lidar(std::vector<Eigen::Vector3d> &point3d, cv::Size boardSize)
{
  /// check point3D
  if(point3d.at(0)[1] > point3d.at(boardSize.width+1)[1]){
    for(int h = 0; h < boardSize.height/2; h++)
    {
      int front = boardSize.width*h;
      int end = boardSize.width*(boardSize.height-1-h);
      for(int w = 0; w < boardSize.width; w++)
      {
          Eigen::Vector3d p = point3d.at(front+w);
          point3d.at(front+w) = point3d.at(end+w);
          point3d.at(end+w) = p;
      }
    }
  }
  if(point3d.at(0)[0] > point3d.at(1)[0]){
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
//  if(point3d.at(0)[2] < point3d.at(boardSize.width+1)[2])
//    std::reverse(point3d.begin(), point3d.end());

//  if(point3d.at(0)[1] < point3d.at(1)[1])
//  {
//    for(int h = 0; h < boardSize.height; h++)
//    {
//      int front = boardSize.width*h;
//      for(int w = 0; w < boardSize.width/2; w++)
//      {
//          Eigen::Vector3d p = point3d.at(front+w);
//          point3d.at(front+w) = point3d.at(front+boardSize.width-1-w);
//          point3d.at(front+boardSize.width-1-w) = p;
//      }
//    }
//  }

}
