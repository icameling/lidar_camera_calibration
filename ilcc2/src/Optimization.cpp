#include "ilcc2/Optimization.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>  /// cv::solvePnPRansac

#include <ilcc2/CeresPnpError.h>        /// CeresPnpError

Optimization::Optimization()
{

}


Eigen::Isometry3d Optimization::solvePose3d2dError(std::vector<Eigen::Vector3d> pts3d,
                                                   std::vector<cv::Point2d> pts2d,
                                                   cv::Mat K,
                                                   Eigen::Vector3d& r_ceres,
                                                   Eigen::Vector3d& t_ceres)

{
    Eigen::Vector4d camera;
    camera(0) = K.at<double>(0,0);
    camera(1) = K.at<double>(0,2);
    camera(2) = K.at<double>(1,1);
    camera(3) = K.at<double>(1,2);

//    Eigen::Vector3d r_ceres(0,0,0);
//    Eigen::Vector3d t_ceres(0,0,0);

//    cv::Mat rvec, tvec;
//    cv::solvePnPRansac( pts3d, pts2d, K, cv::Mat(), rvec, tvec, true, 100, 4.0, 0.99);
//    for(int i = 0; i < 3; i++)
//    {
//        r_ceres(i) = rvec.at<double>(i);
//        t_ceres(i) = tvec.at<double>(i);
//    }

    std::cout << "r_initial: " << r_ceres.transpose() << std::endl;
    std::cout << "t_initial: " << t_ceres.transpose() << std::endl;


    //优化
    ceres::Problem problem;
    for ( unsigned int i=0; i<pts3d.size(); i++ )
    {
        ceres::CostFunction* cost_function = 0;

        cost_function = new ceres::AutoDiffCostFunction<Pose3d2dError, 2, 3, 3>(
                       new Pose3d2dError(pts3d.at(i), pts2d.at(i), camera));

        problem.AddResidualBlock(cost_function,
                                 new ceres::HuberLoss(0.1),
                                 r_ceres.data(),
                                 t_ceres.data());
    }
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

//    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    //options.max_num_iterations = 20;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout << "r_ceres: " << r_ceres.transpose() << std::endl;
    std::cout << "t_ceres: " << t_ceres.transpose() << std::endl;


    Eigen::Vector3d temp = r_ceres;
    double rad = temp.norm();
    temp.normalize();
    Eigen::AngleAxisd rotation_vector ( rad, temp );     //用旋转向量构造旋转向量！


    Eigen::Vector3d euler_angles = rotation_vector.matrix().eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    std::cout<<"yaw pitch roll = "<<(euler_angles*180/3.14).transpose()<<std::endl;


    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector.matrix());
    T.pretranslate(Eigen::Vector3d ( t_ceres[0], t_ceres[1], t_ceres[2]));

//    cout << "Eigen::Isometry3d \n" << T.matrix() << endl;
    return T;
}


void Optimization::get_theta_t(myPointCloud::Ptr cloud,
                               Eigen::Vector2i board_size,
                               Eigen::Vector2d gray_zone,
                               bool topleftWhite,
                               double grid_length,
                               Eigen::Vector3d& theta_t,
                               bool useOutofBoard)
{

  //// theta_t 可以给初值
    Eigen::Vector3i black_gray_white(0,0,0);          /// 统计
    ceres::Problem problem;

    myPoint temp;
    for ( unsigned int i=0; i < cloud->size(); i++ )
    {
        temp = cloud->points[i];
        ceres::CostFunction* cost_function = 0;

        bool laser_white;
        if(temp.intensity < gray_zone[0]) {
            laser_white = false;
            black_gray_white(0)++;
        }
        else if(temp.intensity > gray_zone[1]) {
            laser_white = true;
            black_gray_white(2)++;
        }
        else {
            black_gray_white(1)++;
            continue;
        }

        Eigen::Vector2d laserPoint(temp.y, temp.z); // = point2d.at(i);
        Eigen::Matrix2d sqrtPrecisionMat = Eigen::Matrix2d::Identity();


        cost_function = new ceres::AutoDiffCostFunction<VirtualboardError, 1, 3>(
                       new VirtualboardError(board_size, topleftWhite, grid_length, laser_white,
                                            useOutofBoard, laserPoint, sqrtPrecisionMat));

        //new ceres::CauchyLoss(0.5)
        problem.AddResidualBlock(cost_function,
                                 new ceres::HuberLoss(0.1),
                                 theta_t.data());
    }


//    std::cout << "black_gray_white: " << black_gray_white.transpose() << std::endl;
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = false;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

//    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    //std::cout << theta_t.transpose() << std::endl;
}

void Optimization::get_board_corner(cv::Size boardSize, double squareSize, std::vector<cv::Point3d> &bBoardCorner)
{
  double board_z_axis = 1.1;
  const cv::Point3d pointO(boardSize.width*squareSize/2.0, boardSize.height*squareSize/2.0, board_z_axis);

  for(int i = 0; i < boardSize.height; i++){
      for(int j = 0; j < boardSize.width; j++){
          cv::Point3d p;
          p.x = (j - pointO.x)*squareSize;
          p.y = (i - pointO.y)*squareSize;
          p.z = board_z_axis;
          bBoardCorner.push_back( p );
      }
  }
}

Eigen::Isometry3d Optimization::solvePnP(std::vector<cv::Point2f> corners, cv::Mat camK, cv::Size boardSize, double squareSize)
{
  std::shared_ptr<Camera> camera(new Camera);

  double fx = camK.at<double>(0,0);
  double cx = camK.at<double>(0,2);
  double fy = camK.at<double>(1,1);
  double cy = camK.at<double>(1,2);
  camera->set(fx, fy, cx, cy);


  std::vector<cv::Point3d> bBoardCorner;
  get_board_corner(boardSize, squareSize, bBoardCorner);

  ceres::Problem problem;

  Eigen::Matrix<double,6,1> se3;
  se3 << 0, 0, 0, 0, 0, 0;
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  for(size_t i = 0; i < bBoardCorner.size(); ++i) {

      Eigen::Vector3d pt(bBoardCorner.at(i).x, bBoardCorner.at(i).y, bBoardCorner.at(i).z );
      Eigen::Vector2d uv(corners.at(i).x, corners.at(i).y);

      ceres::CostFunction * costFun = new CeresPnpError(pt, uv, information, camera);
      problem.AddResidualBlock(costFun, new ceres::HuberLoss(0.5), se3.data());

  }

  problem.SetParameterization(se3.data(), new SE3Parameterization());

  ceres::Solver::Options options;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::DOGLEG;
  //options.minimizer_progress_to_stdout = true;
  options.dogleg_type = ceres::SUBSPACE_DOGLEG;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";


  Sophus::SE3 T = Sophus::SE3::exp(se3);

//  std::cout << T.log() << std::endl;    /// 前三维不是位移！！后三维旋转向量
//  cout << T.so3() << endl;    /// 旋转向量
//  Eigen::Vector3d euler_angles = (T.rotation_matrix()).eulerAngles ( 2,1,0 );
//  std::cout << "CeresPnp:" << (euler_angles*180.0/3.14).transpose() <<" "
//            << T.translation().transpose() << std::endl;

  Eigen::Isometry3d T_board2cam = Eigen::Isometry3d::Identity();
  T_board2cam.rotate(T.rotation_matrix());
  T_board2cam.pretranslate(T.translation()); /// 位移

//  std::cout << T_board2cam.matrix() << std::endl;

  return T_board2cam;
}




