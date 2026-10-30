/**
 * 文件职责：差速轮运动积分与整数编码器反馈
 * 输入：两轮行程；输出：双精度模拟位姿、编码器计数。
 * 调用关系：属于 ParkingSimulation 的 PDE 标签页，由 SimulationRunner.Plant 使用。
 */
/** @brief Cached left and right encoder counts. */
class RobotEncoders_c {
  float left = 0;
  float right = 0;
}

/** @brief Robot pose in world coordinates: x/y in mm and heading in radians. */
class RobotPose_c {
  float x = 0;
  float y = 0;
  float theta = 0;
}

/**
 * @brief Ideal differential-drive pose and encoder model.
 * @details The model integrates wheel travel over each fixed simulation step.
 * It makes no claim that its geometry or encoder resolution is calibrated to
 * a physical robot.
 */
class OdometryModel_c {
  final float mmPerEncoderCount;
  RobotEncoders_c encoders = new RobotEncoders_c();
  RobotPose_c pose = new RobotPose_c();
  // Keep continuous simulated wheel counts and pose in double precision.
  // Float public views remain for the original renderer and sensor API only.
  double preciseX, preciseY, preciseTheta, preciseLeft, preciseRight;
  long leftCount() { return Math.round(preciseLeft); }
  long rightCount() { return Math.round(preciseRight); }

  /**
   * @param wheelRadiusMm Wheel radius used to convert revolutions to mm.
   * @param encoderCountsPerRevolution Encoder resolution used for count output.
   */
  OdometryModel_c(float wheelRadiusMm, float encoderCountsPerRevolution) {
    mmPerEncoderCount = TWO_PI * wheelRadiusMm / encoderCountsPerRevolution;
  }

  /** @brief Sets the complete pose and encoder state for a new run. */
  void reset(float x, float y, float theta, float leftEncoder,
    float rightEncoder) {
    preciseX=x; preciseY=y; preciseTheta=theta;
    preciseLeft=leftEncoder; preciseRight=rightEncoder;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    encoders.left = leftEncoder;
    encoders.right = rightEncoder;
  }

  /**
   * @brief Integrates one differential-drive motion increment.
   * @param leftDistance Left wheel travel in mm over the step.
   * @param rightDistance Right wheel travel in mm over the step.
   * @param wheelSeparationMm Distance between wheels in mm.
   */
  void update(float leftDistance, float rightDistance, float wheelSeparationMm) {
    double distance = ((double)leftDistance + rightDistance) / 2.0;
    double deltaTheta = ((double)rightDistance - leftDistance) / wheelSeparationMm;
    // 用当前航向积分行程，保持与已验证仿真相同的计算顺序。
    preciseX += distance * Math.cos(preciseTheta);
    preciseY += distance * Math.sin(preciseTheta);
    preciseTheta = Math.atan2(Math.sin(preciseTheta+deltaTheta),Math.cos(preciseTheta+deltaTheta));
    preciseLeft += (double)leftDistance / mmPerEncoderCount;
    preciseRight += (double)rightDistance / mmPerEncoderCount;
    pose.x=(float)preciseX;pose.y=(float)preciseY;pose.theta=(float)preciseTheta;
    encoders.left=(float)preciseLeft;encoders.right=(float)preciseRight;
  }
}
