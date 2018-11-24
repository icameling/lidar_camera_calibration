#ifndef CERESPNPERROR_H
#define CERESPNPERROR_H

#include <ceres/ceres.h>
#include <sophus/se3.h>


class Camera {
public:
    Camera(){}
    Camera(double fx, double fy, double cx, double cy) {
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
    }

    void set(double fx, double fy, double cx, double cy){
      fx_ = fx;
      fy_ = fy;
      cx_ = cx;
      cy_ = cy;
    }

    Eigen::Vector2d project(double x, double y, double z){
      Eigen::Vector2d uv;
      uv(0) = fx_ * x / z + cx_;
      uv(1) = fy_ * y / z + cy_;

      return uv;
    }
    Eigen::Vector2d project(Eigen::Vector3d point){
      return project(point(0), point(1), point(2));
    }

    double fx_;
    double fy_;
    double cx_;
    double cy_;
};


/**
 * @brief The CeresPnpError class
 * 继承ceres::SizedCostFunction，
 * 声明误差维度、优化量维度1、(优化量维度2、...)
 * 这里只声明了优化量维度1，之后的只有if(jacobians[0])会是true
 */
class CeresPnpError:public ceres::SizedCostFunction<2,6>
{
public:
    CeresPnpError(Eigen::Vector3d& pt, Eigen::Vector2d &uv,
                 Eigen::Matrix<double, 2, 2> &information,
                  std::shared_ptr<Camera> cam) : pt_(pt), uv_(uv), cam_(cam){
        //printf("index = %d\n", index++);
        Eigen::LLT<Eigen::Matrix<double, 2, 2>> llt(information);
        sqrt_information_ = llt.matrixL();
    }
    virtual ~CeresPnpError() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
        Sophus::SE3 T = Sophus::SE3::exp(lie);

        //std::cout << T.matrix3x4() << std::endl;

        Eigen::Vector3d P = T * pt_;
        Eigen::Vector2d uv = cam_->project(P);
        Eigen::Vector2d err = uv - uv_;
        err = sqrt_information_ * err;

        residuals[0] = err(0);
        residuals[1] = err(1);

        Eigen::Matrix<double, 2, 6> Jac = Eigen::Matrix<double, 2, 6>::Zero();
        Jac(0, 0) = cam_->fx_ / P(2);
        Jac(0, 2) = -P(0) / P(2) /P(2) * cam_->fx_;
        Jac(0, 3) = Jac(0, 2) * P(1);
        Jac(0, 4) = cam_->fx_ - Jac(0, 2) * P(0);
        Jac(0, 5) = -Jac(0, 0) * P(1);

        Jac(1, 1) = cam_->fy_ / P(2);
        Jac(1, 2) = -P(1) / P(2) /P(2) * cam_->fy_;
        Jac(1, 3) = -cam_->fy_ + Jac(1, 2) * P(1);
        Jac(1, 4) = -Jac(1, 2) * P(0);
        Jac(1, 5) = Jac(1, 1) * P(0);

        Jac = sqrt_information_ * Jac;

        int k = 0;
        for(int i = 0; i < 2; ++i) {
            for(int j = 0; j < 6; ++j)
            {
                if(k >= 12)
                    return false;
                if(jacobians) {
                    if(jacobians[0])
                        jacobians[0][k] = Jac(i, j);
                }
                k++;
            }
        }

        //printf("jacobian ok!\n");

        return true;
    }

public:
    Eigen::Vector3d pt_;
    Eigen::Vector2d uv_;
    std::shared_ptr<Camera> cam_;
    Eigen::Matrix<double, 2, 2> sqrt_information_;
    static int index;
};

int CeresPnpError::index = 0;



class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
    SE3Parameterization() {}
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool SE3Parameterization::Plus(const double* x,
                  const double* delta,
                  double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

    Sophus::SE3 T = Sophus::SE3::exp(lie);
    Sophus::SE3 delta_T = Sophus::SE3::exp(delta_lie);
    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();

    for(int i = 0; i < 6; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);

    return true;

}


#endif // CERESPNPERROR_H
