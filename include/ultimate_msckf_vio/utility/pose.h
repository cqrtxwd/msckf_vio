#ifndef UTILITY_POSE_H
#define UTILITY_POSE_H
#include <eigen3/Eigen/Eigen>

namespace ultimate_msckf_vio {

template <typename FloatType>
class Pose3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using Matrix = Eigen::Matrix<FloatType, 3, 3>;


  Pose3():rotation_(Quaternion::Identity()), translation_(Vector::Zero()) {}

  Pose3(const Quaternion& rotation,
        const Vector& translation)
    :rotation_(rotation), translation_(translation) {}

  Quaternion rotation() const {
    return rotation_;
  }

  Vector translation() const {
    return translation_;
  }

  void SetRotation(Quaternion rotation) {
    rotation_ = rotation;
  }

  void SetRotation(Matrix R) {
    rotation_ = Quaternion(R);
  }

  void SetTranslation(Vector translation) {
    translation_ = translation;
  }

  Pose3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

 private:
  Quaternion rotation_;
  Vector translation_;
};

template <typename FloatType>
Pose3<FloatType> operator*(const Pose3<FloatType>& lhs,
                           const Pose3<FloatType>& rhs) {
  return Pose3<FloatType>(
        (lhs.rotation() * rhs.rotation()).normalized(),
        lhs.rotation() * rhs.translation() + lhs.translation());
}

typedef Pose3<double> Pose3d;
typedef Pose3<float> Pose3f;

}

#endif // UTILITY_POSE_H
