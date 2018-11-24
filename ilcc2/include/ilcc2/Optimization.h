#ifndef OPTIMIZATION_H_
#define OPTIMIZATION_H_

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>     /// ceres::AngleAxisRotatePoint
#include <opencv2/core.hpp>

#include "ilcc2/config.h"


class VirtualboardError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VirtualboardError(const Eigen::Vector2i& board_size,
                       const bool topleftWhite,
                       const double grid_length,
                       const bool laser_white,
                       const bool useOutofBoard,
                       const Eigen::Vector2d& laserPoint,
                       const Eigen::Matrix2d& sqrtPrecisionMat):
          m_board_size(board_size),
          m_topleftWhite(topleftWhite),
          m_grid_length(grid_length),
          m_laser_white(laser_white),
          m_useOutofBoard(useOutofBoard),
          m_laserPoint(laserPoint),
          m_sqrtPrecisionMat(sqrtPrecisionMat) {}
    template <typename T>
    bool operator()(const T* const theta_t,
                    T* residuals) const
    {
        Eigen::Matrix<T,2,1> P = m_laserPoint.cast<T>();

        const T angle_axis[3] = {theta_t[0], T(0), T(0)};
        const T pt3[3] = {T(0), P(0), P(1)};
        T result[3];
        ceres::AngleAxisRotatePoint(angle_axis, pt3, result);
        result[1] = result[1] + theta_t[1];
        result[2] = result[2] + theta_t[2];


        T i = (result[1]+T(m_board_size(0))*T(m_grid_length)/T(2.0)) / T(m_grid_length);
        T j = (result[2]+T(m_board_size(1))*T(m_grid_length)/T(2.0)) / T(m_grid_length);
        // in board
        if( i > T(0) && i < T(m_board_size(0)) &&
            j > T(0) && j < T(m_board_size(1)) )
        {
            T ifloor = floor ( i );
            T jfloor = floor ( j );

            T ii = floor(ifloor / T(2)) *T(2);
            T jj = floor(jfloor / T(2)) *T(2);

            bool White = !m_topleftWhite;
            if(ifloor==ii && jfloor==jj)  // 都是偶数
                White = m_topleftWhite;
            if(ifloor!=ii && jfloor!=jj)  // 都是奇数
                White = m_topleftWhite;

            /// 颜色一致，无误差
            if(m_laser_white == White){
                residuals[0] = T(0);
            }
            else {
                T iceil = ceil ( i );
                T jceil = ceil ( j );
                T ierror, jerror;
                if(i-ifloor > T(0.5))
                    ierror = iceil - i;
                else
                    ierror = i-ifloor;

                if(j-jfloor > T(0.5))
                    jerror = jceil - j;
                else
                    jerror = j-jfloor;

                residuals[0] = ierror + jerror;
            }

        }
        // out of board
        else if(m_useOutofBoard){

            T ierror; /*= min( abs(i-T(0)) , abs(i - T(m_board_size(0))));*/
            T jerror; /*= min( abs(j-T(0)) , abs(j - T(m_board_size(1))));*/

            if( abs(i-T(0)) < abs(i - T(m_board_size(0))) )
              ierror = abs(i-T(0));
            else
              ierror = abs(i - T(m_board_size(0)));

            if( abs(j-T(0)) < abs(j - T(m_board_size(1))) )
              jerror = abs(j-T(0));
            else
              jerror = abs(j - T(m_board_size(1)));

            residuals[0] = ierror + jerror;
        } else {
          residuals[0] = T(0);
        }

        return true;
    }
private:

    Eigen::Vector2i m_board_size;  // width height
    bool m_topleftWhite;
    double m_grid_length;
    bool m_laser_white;

    bool m_useOutofBoard;

    Eigen::Vector2d m_laserPoint;

    // square root of precision matrix
    Eigen::Matrix2d m_sqrtPrecisionMat; // 误差项的权重
};




class Pose3d2dError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose3d2dError(cv::Point3d pt3d, cv::Point2d pt2d, Eigen::Vector4d camera):
                    pt3d_(pt3d), pt2d_(pt2d), camera_(camera){}

    Pose3d2dError(Eigen::Vector3d pt3d, cv::Point2d pt2d, Eigen::Vector4d camera):
                    pt2d_(pt2d), camera_(camera){
        pt3d_.x = pt3d(0);
        pt3d_.y = pt3d(1);
        pt3d_.z = pt3d(2);
    }

    Pose3d2dError(cv::Point3d pt3d, cv::Point2d pt2d, cv::Mat camK):
                    pt3d_(pt3d), pt2d_(pt2d){
        camera_(0) = camK.at<double>(0,0);
        camera_(1) = camK.at<double>(0,2);
        camera_(2) = camK.at<double>(1,1);
        camera_(3) = camK.at<double>(1,2);
    }

    template <typename T>
    bool operator() (const T* const r, // 旋转向量 3d
                     const T* const t, // 平移向量 3d
                     T* residuals) const
    {
        T predicted_pt[3];//预测值
        T pt3[3];
        pt3[0] = T(pt3d_.x);
        pt3[1] = T(pt3d_.y);
        pt3[2] = T(pt3d_.z);
        ceres::AngleAxisRotatePoint(r, pt3, predicted_pt);
        predicted_pt[0] += t[0];
        predicted_pt[1] += t[1];
        predicted_pt[2] += t[2];

        predicted_pt[0] = predicted_pt[0] / predicted_pt[2];
        predicted_pt[1] = predicted_pt[1] / predicted_pt[2];

        //像素点转到归一化坐标
//        predicted_pt[0] = T(camera_->fx_)*predicted_pt[0] + T(camera_->cx_);
//        predicted_pt[1] = T(camera_->fy_)*predicted_pt[1] + T(camera_->cy_);
        predicted_pt[0] = T(camera_[0])*predicted_pt[0] + T(camera_[1]);
        predicted_pt[1] = T(camera_[2])*predicted_pt[1] + T(camera_[3]);


        //预测 - 观测
        residuals[0] = T(pt2d_.x) - predicted_pt[0];
        residuals[1] = T(pt2d_.y) - predicted_pt[1];

//        cout << "2residuals: " << residuals[0] << " " << residuals[1] << endl;

        //cout << "residuals:"<<residuals[0]+residuals[1]<<endl;
        return true;
    }

private:
    cv::Point3d pt3d_;
    cv::Point2d pt2d_;
    Eigen::Vector4d camera_; // 分别是fx,cx,fy,cy

};



class Optimization
{
public:
    Optimization();

    Eigen::Isometry3d solvePose3d2dError(std::vector<Eigen::Vector3d> pts3d,
                                         std::vector<cv::Point2d> pts2d,
                                         cv::Mat K,
                                         Eigen::Vector3d& r_ceres,
                                         Eigen::Vector3d& t_ceres);

    void  get_theta_t(myPointCloud::Ptr cloud,
                                   Eigen::Vector2i board_size,
                                   Eigen::Vector2d gray_zone,
                                   bool topleftWhite,
                                   double grid_length,
                                   Eigen::Vector3d& theta_t,
                                   bool useOutofBoard = true);

    void get_board_corner(cv::Size boardSize, double squareSize, std::vector<cv::Point3d> &bBoardCorner);
    Eigen::Isometry3d solvePnP(std::vector<cv::Point2f> corners, cv::Mat camK, cv::Size boardSize, double squareSize);





};




#endif // OPTIMIZATION_H
