#ifndef EKF_H
#define EKF_H
#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <utility>
#define PRINT_MATRIX(x) std::cout << #x "=\n" << x << std::endl


using Quaterniond = Eigen::Quaterniond;
using Vec3d = Eigen::Vector3d;


template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &v3d) {
  typedef typename Derived::Scalar Scalar_t;

  const double kAngleEpisode = 1e-6;
  double theta = v3d.norm();
  double half_theta = 0.5 * theta;

  double imag_factor;
  double real_factor = cos(half_theta);
  if (theta < kAngleEpisode) {
    double theta_sq = theta * theta;
    double theta_po4 = theta_sq * theta_sq;
    // taylor expansion of sin(t/2)/t, visit https://www.wolframalpha.com/input/?i=sin%28t%2F2%29%2Ft for reference.
    imag_factor = 0.5 - (1 / 48.) * theta_sq + (1 / 3840.) * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
  }

  // return {cos(|t|/2), sin(|t|/2)/|t|*t}
  return Eigen::Quaterniond(real_factor, imag_factor * v3d.x(), imag_factor * v3d.y(), imag_factor * v3d.z())
      .cast<Scalar_t>();
}

inline Eigen::Matrix3d Skew(const Vec3d &t) {
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
    return t_hat;
}

// eigen quaternion memory layout is xyzw instead of wxyz
template<typename T>
Eigen::Matrix<T, 4, 4> Ql(const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q.w() + Skew(q.vec());
    ans.template block<3, 1>(0, 3) = q.vec();
    ans.template block<1, 3>(3, 0) = -q.vec();
    ans(3, 3) = q.w();
    return ans;
}

template<typename T>
Eigen::Matrix<T, 4, 4> Qr(const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q.w() - Skew(q.vec());
    ans.template block<3, 1>(0, 3) = q.vec();
    ans.template block<1, 3>(3, 0) = -q.vec();
    ans(3, 3) = q.w();
    return ans;
}


class Estimator {
public:
  Estimator() : init_ok(false) {}

  virtual ~Estimator() = default;

  virtual Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                   const Vec3d &linear_acceleration) = 0;

public:
  double get_last_timestamp() const { return last_timestamp; }

  bool get_init_ok() const { return init_ok; }

  void InitPoseByGravity(const Vec3d &g) {
    // warning: FromTwoVectors() should be used as a static function
    this->pose = Quaterniond::FromTwoVectors(g, Vec3d::UnitZ());
    this->init_ok = true;
  }

protected:
  bool init_ok;
  Quaterniond pose;
  double last_timestamp;
};

class EkfEstimator : public Estimator {
public:
  Quaterniond EstimatePose(double timestamp, const Vec3d &angular_velocity, const Vec3d &linear_acceleration) override;

private:
  Eigen::Matrix4d P;
  Vec3d g_w;

  // TODO fine-tune progress noise and measurement noise here.
  const Eigen::Matrix4d R = Eigen::Matrix4d::Identity() * 0.1;
  const Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.1;
};

struct CostFunctor {
private:
  explicit CostFunctor(Vec3d g) : g_(std::move(g)) {}

public:
  template <typename T> bool operator()(const T *const x, T *residual) const {
    Eigen::Quaternion<T> q{x};
    Eigen::Map<Eigen::Matrix<T, 3, 1>>{residual} = q * g_.template cast<T>();
    return true;
  }

  static ceres::CostFunction *Create(const Vec3d &g) {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 4>(new CostFunctor(g));
  }

private:
  Vec3d g_;
};

//Returns the Jacobian of the measurement equation w.r.t. the quaternion
Eigen::Matrix<double, 3, 4> GetMeasureEquationJacobian(const Quaterniond &q, const Vec3d &g) {
  auto f = CostFunctor::Create(g);

  std::vector<const double *> pb;
  pb.push_back(q.coeffs().data());

  double null_placeholder[100];

  std::vector<double *> jacobians;
  Eigen::Matrix<double, 3, 4> J;
  for (int i = 0; i < J.rows(); ++i) {
    jacobians.push_back(&J(i, 0));
  }

  // This code gets timed
  f->Evaluate(pb.data(), null_placeholder, jacobians.data());

  return J;
}

Quaterniond EkfEstimator::EstimatePose(double timestamp, const Vec3d &angular_velocity,
                                       const Vec3d &linear_acceleration) {
  // normalize acc and consider it as gravity vector directly
  auto acc_norm = linear_acceleration.normalized();
  this->g_w = acc_norm;

  double dt = timestamp - this->last_timestamp;
  this->last_timestamp = timestamp;
  if (!this->init_ok) {
    this->InitPoseByGravity(acc_norm);
    this->P.setZero();
    return this->pose;
  }

  auto delta_theta = dt * angular_velocity;
  auto dq = deltaQ(delta_theta);
  Eigen::Matrix<double, 4, 4> F = Qr(dq);

  // predict
  Quaterniond x_prior = this->pose * dq;
  x_prior.normalize();
  // std::cout << "A: " << x_prior.coeffs().transpose() << std::endl;
  // std::cout << "B: " << F * this->pose.coeffs() << std::endl;
  Eigen::Matrix<double, 4, 4> P_prior = F * this->P * F.transpose() + R;

  // update
  // todo kk do not use inverse()
  Eigen::Matrix<double, 3, 4> H = GetMeasureEquationJacobian(x_prior, this->g_w);
  Eigen::Matrix<double, 4, 3> K = P_prior * H.transpose() * (H * P_prior * H.transpose() + Q).inverse();
  Eigen::Matrix<double, 4, 1> x_posterior =
      x_prior.coeffs() + K * (Eigen::Vector3d::UnitZ() - x_prior.inverse() * this->g_w);
  Eigen::Matrix<double, 4, 4> P_posterior = (Eigen::Matrix4d::Identity() - K * H) * P_prior;

  this->pose = x_posterior;
  this->pose.normalize();
  this->P = P_posterior;

  return this->pose;
}


#endif