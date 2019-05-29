#include "ultimate_msckf_vio/msckf_estimator.h"

namespace ultimate_msckf_vio {

void MsckfEstimator::ProcessMeasurementsInitialization(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("Initializing");
  frame_count_++;
  if (frame_count_ >= 3) {
    ROS_INFO_STREAM("Initialize done ...");
    // TODO: fix this;
    Quaterniond q(1,0,0,0);
    Vector3d bg(1, 1, 1);
    Vector3d v(2, 2, 2);
    Vector3d ba(3, 3, 3);
    Vector3d p(4, 4, 4);

    SetInitialState(q, bg, v, ba, p);
    ROS_INFO_STREAM("set cov");
    ekf_state_.SetCovariance();
    ROS_INFO_STREAM("set cov done");
    estimator_status_ = EstimatorStatus::kNormalStage;
  }
}

void MsckfEstimator::ProcessMeasurementsNormalStage(
    const SensorMeasurement& measurement) {
  LOG(INFO) << "normal stage";
  CHECK(measurement.image->header.stamp ==
             measurement.imu_measurements.back()->header.stamp);
  // propagete imu data, push ekf state to next image timestamp
  ROS_INFO_STREAM("normal stage2 ");
  deque<ImuConstPtr> imu_measurements = measurement.imu_measurements;
//  for (auto imu_msg : imu_measurements) {
//    PropagateImu(imu_msg);
//  }

  // manage feature points
  // we use front-end od VINS, so here is how to restore feature information
  // from VINS feature_tracker
  auto img_msg = measurement.image;
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> cur_image;
  // now image is only a vector with a lot of feature points info,
  constexpr int NUM_OF_CAM = 1;
  for (unsigned int i = 0; i < img_msg->points.size(); i++)
  {
      int v = img_msg->channels[0].values[i] + 0.5;
      int feature_id = v / NUM_OF_CAM;
      int camera_id = v % NUM_OF_CAM;  // alaways = 0
      double x = img_msg->points[i].x;
      double y = img_msg->points[i].y;
      double z = img_msg->points[i].z;
      double p_u = img_msg->channels[1].values[i];
      double p_v = img_msg->channels[2].values[i];
      double velocity_x = img_msg->channels[3].values[i];
      double velocity_y = img_msg->channels[4].values[i];
      CHECK(z == 1);
      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
      cur_image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//      ROS_INFO_STREAM("v: "<<v;);
//      ROS_INFO_STREAM("feature_id: "<<feature_id;);
  }


  // feature_manager add data

  // check keyframe

  if (KeyFrameCheck()) {
    // add imu state to KF state
    // build a keyframe and build feature bundles of each point
    visual_observation_manager_.AddNewKeyFrame(cur_image);

    // after add features to manager, try Find all the feature bundle ready
    // to optimize
    Timer check_bundle_timer;
    check_bundle_timer.StartTimer();
    vector<FeatureBundle> completed_feature_bundles;
    visual_observation_manager_.FindCompletedFeatureBundles(
          &completed_feature_bundles);
    LOG(INFO) << "now ready to optimze bundle num: "
              << completed_feature_bundles.size();

    auto time_cost = check_bundle_timer.EndTimer();
    LOG(INFO) << "FindCompletedFeatureBundles cost : " << time_cost << "secs";

    // ekf_state and cov agument
    ekf_state_.AugmentStateAndCovariance();

    // check if reach max window size, we marginalize the oldest keyframe, set
    // all the feature bundle involved in this keyframe ready to optimize.
    // there is better strategy to marginalize, but for now, we just simplely
    // marginalize the oldest
    if(visual_observation_manager_.ReachMaximumWindowSize()) {
      visual_observation_manager_.BuildFeatureBundleFromOldestKeyframe(
            &completed_feature_bundles);
      LOG(INFO) << "now ready to optimze bundle num: "
                << completed_feature_bundles.size();
    }

    // if there are feature bundles build complete, ekf update
    if (!completed_feature_bundles.empty()) {
      ekf_update_->AddVisualConstraints(&completed_feature_bundles);
      if (!ekf_update_->IEKFUpdate(&ekf_state_)) {
        LOG(FATAL) << "IEKFUpdate fail";
      }
    }


    // TODO: after ekf update, maginlize keyframe
    if(ShouldMarginalize()) {
      visual_observation_manager_.MarginalizeOldestKeyframe();
    }



    // when KFs in slide window reaches Maximun size , ekf update
//    ekf_update_->IEKFUpdate();


  }
  frame_count_++;
}



void MsckfEstimator::PropagateImu(const sensor_msgs::ImuConstPtr& imu_msg) {
  ROS_INFO_STREAM("ProcessImu ");
  if (cur_time_ <= 0) {
    cur_time_ = imu_msg->header.stamp.toSec();
    pre_time_ = cur_time_;
    return;
  }
  pre_time_ = cur_time_;
  cur_time_ = imu_msg->header.stamp.toSec();

  // convert ros sensor_msg to Eigen Vector
  Eigen::Vector3d accel_data(imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyro_data(imu_msg->angular_velocity.x,
                            imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z);

  // low pass filter for imu data
  auto filtered_accel_data = accel_filter_.FilterData(accel_data, cur_time_);
  auto fitered_gyro_data = gyro_filter_.FilterData(gyro_data, cur_time_);
  // propagate imu to next timestamp
  double delta_t = cur_time_ - pre_time_;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> phi;
  phi.setIdentity(kImuErrorStateSize, kImuErrorStateSize);
  if (!PropagateEkfStateAndPhiByRungeKutta(filtered_accel_data,
                                           fitered_gyro_data,
                                           delta_t,
                                           &ekf_state_,
                                           &phi)) {
    ROS_INFO_STREAM("PropagateEkfStateAndPhiByRungeKutta failed !!!");
  }
}

bool MsckfEstimator::PropagateEkfStateAndPhiByRungeKutta(
    const Vector3d& accel_measurement,
    const Vector3d& gyro_measurement,
    const double& delta_t,
    EkfState<double>* ekf_state,
    Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* phi) {
  if (ekf_state == nullptr || phi == nullptr) {
    return false;
  }
  ROS_INFO_STREAM("PropagateEkfState ");
  // get imu data in robot_body_frame(IMU frame)
  Vector3d gyro_body = gyro_measurement - ekf_state->imu_state.bg();
  Vector3d accel_body = accel_measurement - ekf_state->imu_state.ba();
  Vector3d gravity(0, 0, 9.8);
  // use RungeKutta4 to propagate IMU state
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> origin_phi;
  origin_phi.setIdentity(kImuErrorStateSize, kImuErrorStateSize);
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> imu_covariance =
      ekf_state->GetImuStateCovariance();
  Matrix<double, kImuStateSize, 1> state_vector
      = ekf_state->GetImuStateVector();
  Matrix<double, kImuStateSize, 1> k1_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k1_phi_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k2_phi;
  Matrix<double, kImuStateSize, 1> k2_state;
  Matrix<double, kImuStateSize, 1> k2_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k2_phi_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k3_phi;
  Matrix<double, kImuStateSize, 1> k3_state;
  Matrix<double, kImuStateSize, 1> k3_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k3_phi_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k4_phi;
  Matrix<double, kImuStateSize, 1> k4_state;
  Matrix<double, kImuStateSize, 1> k4_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> k4_phi_derivative;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> final_phi_derivative;
  Matrix<double, kImuStateSize, 1> final_derivative;
  Matrix<double, kImuStateSize, 1> final_state;

  // 0 ---- t/2 ---- 1
  // k1    k2 k3    k4
  // compute k1_derivative
  if(!PropagateStateByRungeKuttaSingleStep(state_vector,
                                           origin_phi,
                                           accel_body,
                                           gyro_body,
                                           gravity,
                                           delta_t / 2,
                                           &k1_derivative,
                                           &k1_phi_derivative,
                                           &imu_covariance)) {
    ROS_INFO_STREAM("PropagateStateAndCovByRungeKutta k1 failed !!!");
  }

  // compute k2_derivative
  k2_state = state_vector + delta_t / 2 * k1_derivative;
  k2_phi = origin_phi + delta_t / 2 * k1_phi_derivative;
  if(!PropagateStateByRungeKuttaSingleStep(k2_state,
                                           k2_phi,
                                           accel_body,
                                           gyro_body,
                                           gravity,
                                           delta_t / 2,
                                           &k2_derivative,
                                           &k2_phi_derivative,
                                           &imu_covariance)) {
    ROS_INFO_STREAM("PropagateStateAndCovByRungeKutta k2 failed !!!");
  }

  // compute k3_derivative
  k3_state = state_vector + delta_t / 2 * k2_derivative ;
  k3_phi = k2_phi + delta_t / 2 * k2_phi_derivative;
  if(!PropagateStateByRungeKuttaSingleStep(k3_state,
                                           k3_phi,
                                           accel_body,
                                           gyro_body,
                                           gravity,
                                           delta_t / 2,
                                           &k3_derivative,
                                           &k3_phi_derivative,
                                           &imu_covariance)) {
    ROS_INFO_STREAM("PropagateStateAndCovByRungeKutta k3 failed !!!");
  }

  // compute k4_derivative
  k4_state = state_vector + delta_t * k3_derivative;
  k4_phi = origin_phi + delta_t * k3_phi_derivative;
  if(!PropagateStateByRungeKuttaSingleStep(k4_state,
                                           k4_phi,
                                           accel_body,
                                           gyro_body,
                                           gravity,
                                           delta_t,
                                           &k4_derivative,
                                           &k4_phi_derivative,
                                           &imu_covariance)) {
    ROS_INFO_STREAM("PropagateStateAndCovByRungeKutta k4 failed !!!");
  }
  final_derivative = (k1_derivative +
                      2 * k2_derivative +
                      2 * k3_derivative +
                      k4_derivative) / 6;
  final_state = state_vector + delta_t * final_derivative;
  ekf_state->SetImuStateVector(final_state);
  ROS_INFO_STREAM("final_state:  " << final_state);

  final_phi_derivative = (k1_phi_derivative +
                          2 * k2_phi_derivative +
                          2 * k3_phi_derivative +
                          k4_phi_derivative) / 6;
  *phi = origin_phi + delta_t * final_phi_derivative;

  return true;
}

void MsckfEstimator::SetInitialState(const Eigen::Quaterniond& q,
                                     const Eigen::Vector3d& bg,
                                     const Eigen::Vector3d& v,
                                     const Eigen::Vector3d& ba,
                                     const Eigen::Vector3d& p) {
  Matrix<double, 16, 1> init_state_vector;
  init_state_vector << q.coeffs(), bg, v, ba, p;
  ekf_state_.SetImuStateVector(init_state_vector);
}

bool MsckfEstimator::PropagateStateByRungeKuttaSingleStep(
    const Matrix<double, kImuStateSize, 1>& state_vector,
    const Matrix<double, kImuErrorStateSize, kImuErrorStateSize>& state_transition,
    const Matrix<double, 3, 1>& accel_body,
    const Matrix<double, 3, 1>& gyro_body,
    const Matrix<double, 3, 1>& gravity,
    const double& delta_t,
    Matrix<double, kImuStateSize, 1>* derivative_vector,
    Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* state_transition_derivative,
    Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* covariance) {
  // check input
  if(derivative_vector == nullptr || state_transition_derivative == nullptr) {
    return false;
  }
  // compute state derivative
    Matrix<double, 4, 4> omega;
    Matrix3d w_skew =
        GeometricKit<double>::VectorToSkewSymmetricMatrix(gyro_body);
    omega << - w_skew, gyro_body,
             - gyro_body.transpose(), 0;
    Matrix<double, 4, 1> I_q_G_vector(state_vector.head(4));
    Matrix<double, 4, 1> q_derivative = 0.5 * omega * I_q_G_vector;
    Vector3d bg_derivative = Vector3d(0, 0, 0);
    // ignore earth's self_rotation
    Matrix3d I_R_G =
        GeometricKit<double>::QuatnionVectorToEigenQuaterion(
          state_vector.head(4)).normalized().toRotationMatrix();
    Vector3d v_derivative = I_R_G.transpose() * accel_body + gravity;
    Vector3d ba_derivative = Vector3d(0, 0, 0);
    Vector3d p_derivative = state_vector.block<3, 1>(kVStateIndex, 0);

    derivative_vector->block<4, 1>(kQStateIndex, 0) = q_derivative;
    derivative_vector->block<3, 1>(kBgStateIndex, 0) = bg_derivative;
    derivative_vector->block<3, 1>(kVStateIndex, 0) = v_derivative;
    derivative_vector->block<3, 1>(kBaStateIndex, 0) = ba_derivative;
    derivative_vector->block<3, 1>(kPStateIndex, 0) = p_derivative;
//    ROS_INFO_STREAM("derivative "<< derivative_vector->transpose(););

    // propagate state_transition_matrix
    Matrix<double, kImuErrorStateSize, kImuErrorStateSize> F;
    F.setZero(kImuErrorStateSize, kImuErrorStateSize);
    F.block<3, 3>(kQErrorStateIndex, kQErrorStateIndex) = - w_skew;
    F.block<3, 3>(kQErrorStateIndex, kBgErrorStateIndex)
        = - Eigen::Matrix3d::Identity();
    Matrix3d accel_skew =
        GeometricKit<double>::VectorToSkewSymmetricMatrix(accel_body);
    F.block<3, 3>(kVErrorStateIndex, kQErrorStateIndex) =
        - I_R_G.transpose() * accel_skew;
    F.block<3, 3>(kVErrorStateIndex, kBaErrorStateIndex) = - I_R_G.transpose();
    F.block<3, 3>(kPErrorStateIndex, kVErrorStateIndex) =
        Eigen::Matrix3d::Identity();

    *state_transition_derivative = F * state_transition;

    // propagate cov_derivative
//    Matrix<>



    return true;
}

bool MsckfEstimator::KeyFrameCheck() {
  // for now, we just sample the image in a certain rate
  if (frame_count_ % 3 == 0) {
    return true;
  }
  return false;
}

bool MsckfEstimator::ShouldMarginalize() {
  return visual_observation_manager_.ShouldMarginalize();
}

// we impose ekf update when any of the situations below happens :
// 1) keyframe window size reaches the maximun size;
// 2) at least one feature in window no longer been observed
bool MsckfEstimator::ShouldVisualUpdate() {
  if (visual_observation_manager_.ReachMaximumWindowSize()) {
    return true;
  }
  // not finish

  return false;

}


}  // namespace ultimate_msckf_vio
