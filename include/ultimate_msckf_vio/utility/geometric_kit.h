#ifndef GEOMETRIC_KIT_H_
#define GEOMETRIC_KIT_H_

#include <eigen3/Eigen/Eigen>

namespace ultimate_msckf_vio {
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaternion;

template <typename Scalar>
class GeometricKit {
 public:
  static Matrix<Scalar, 3, 3> VectorToSkewSymmetricMatrix(
      const Matrix<Scalar, 3, 1>& vector) {
    Matrix<Scalar, 3, 3> skew_symmetric_matrix;
    skew_symmetric_matrix << 0, - vector(2), vector(1),
                             vector(2), 0, - vector(0),
                             - vector(1), vector(0), 0;
    return skew_symmetric_matrix;
  }

  static Quaternion<Scalar> QuatnionVectorToEigenQuaterion(
      const Matrix<Scalar, 4, 1>& quatnion_vector) {
    Quaternion<Scalar> result_quaternion(quatnion_vector(3),
                                         quatnion_vector(0),
                                         quatnion_vector(1),
                                         quatnion_vector(2));
    return result_quaternion;
  }
};

template <typename Scalar>
static Matrix<Scalar, 3, 3> VectorToSkewSymmetricMatrix(
    const Matrix<Scalar, 3, 1>& vector) {
  Matrix<Scalar, 3, 3> skew_symmetric_matrix;
  skew_symmetric_matrix << 0, - vector(2), vector(1),
                           vector(2), 0, - vector(0),
                           - vector(1), vector(0), 0;
  return skew_symmetric_matrix;
}


}

#endif // GEOMETRIC_KIT_H_
