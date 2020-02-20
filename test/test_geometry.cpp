#include "eigen3/Eigen/Eigen"
#include "glog/logging.h"
#include "ultimate_msckf_vio/utility/geometric_kit.h"

using namespace ultimate_msckf_vio;
using namespace Eigen;
using ultimate_msckf_vio::VectorToSkewSymmetricMatrix;
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "test geometry ";

  LOG(WARNING) << "-------------- delta rotation test --------------";
  // d_R = I + [dth]^
  double dthx = 0.001;
  double dthy = 0.002;
  double dthz = 0.003;
  Quaterniond delta_q(1, dthx/2, dthy/2, dthz/2);
  delta_q.normalize();

  Matrix3d delta_R = delta_q.toRotationMatrix();
  LOG(INFO) << "delta_R: \n"
            << delta_R;
  LOG(INFO) << "I + [dth]^ \n"
            << Matrix3d::Identity() + VectorToSkewSymmetricMatrix(Vector3d(dthx, dthy, dthz));

  // vec1 x vec2 = - vec2 x vec1
  // aka: [vec1]^ * vec2 = - [vec2]^ * vec1
  LOG(WARNING) << "-------------------- cross product test --------------------";
  Vector3d vec1(4, 5, 6);
  Vector3d vec2(10, 2, 3);

  Vector3d res1 = VectorToSkewSymmetricMatrix(vec1) * vec2;
  Vector3d res2 = - VectorToSkewSymmetricMatrix(vec2) * vec1;
  LOG(INFO) << "[vec1]^ * vec2 =  \n"
            << res1.transpose();
  LOG(INFO) << "- [vec2]^ * vec1 =  \n"
            << res2.transpose();

//  Matrix3d R;
//  R << 1,2,3,
//      4,5,6,
//      7,8,9;
//  Vector3d tmp_vec = R * vec1;
//  Matrix3d res_R1 = VectorToSkewSymmetricMatrix(tmp_vec);
//  Matrix3d res_R2 = R * VectorToSkewSymmetricMatrix(vec1);
//  LOG(INFO) << "\n"
//            <<res_R1 << "\n"
//            << res_R2;






  return 0;
}
