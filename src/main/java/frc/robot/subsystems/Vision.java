// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private static PhotonCamera frontCamera;
  private static PhotonCamera leftCamera;
  private static PhotonCamera rightCamera;
  private static PhotonPoseEstimator frontCamEstimator;
  private static PhotonPoseEstimator leftCamEstimator;
  private static PhotonPoseEstimator rightCamEstimator;
  private static boolean allCamsConnected;

  /** Creates a new Vision. */
  public Vision() {
    frontCamera = new PhotonCamera("frontCamera");
    leftCamera = new PhotonCamera("leftCamera");
    rightCamera = new PhotonCamera("rightCamera");
    frontCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCamera,
        VisionConstants.kFrontCamtoRobot);
    leftCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftCamera,
        VisionConstants.kLeftCamtoRobot);
    rightCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightCamera,
        VisionConstants.kRightCamtoRobot);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/FrontCam Connected", frontCamera.isConnected());
    SmartDashboard.putBoolean("Vision/LeftCam Connected", leftCamera.isConnected());
    SmartDashboard.putBoolean("Vision/RightCam Connected", rightCamera.isConnected());
    allCamsConnected = (frontCamera.isConnected() && leftCamera.isConnected() && rightCamera.isConnected());
  }

  public PhotonPipelineResult getFrontCameraResult() {
    return frontCamera.getLatestResult();
  }

  public PhotonPipelineResult getLeftCameraResult() {
    return leftCamera.getLatestResult();
  }

  public PhotonPipelineResult getRightCameraResult() {
    return rightCamera.getLatestResult();

  }

  public static Optional<EstimatedRobotPose> getFrontCamPoseEst() {
    var visionest = frontCamEstimator.update();
    return visionest;
  }

  public static Optional<EstimatedRobotPose> getLeftCamPoseEst() {
    var visionest = leftCamEstimator.update();
    return visionest;
  }

  public static Optional<EstimatedRobotPose> getRightCamPoseEst() {
    var visionest = rightCamEstimator.update();
    return visionest;
  }

  public static Matrix<N3, N1> getLeftEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kLeftCamSingleStdDevs;
    var targets = leftCamera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = leftCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.kLeftCamMultiStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public static Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kFrontCamSingleStdDevs;
    var targets = leftCamera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = leftCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.kFrontCamMultiStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public static Matrix<N3, N1> getRightEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kRightCamSingleStdDevs;
    var targets = leftCamera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = leftCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.kRightCamMultiStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public static boolean checkConnection() {
    return allCamsConnected;
  }

  public static boolean atSpeaker() {
    var targets = frontCamera.getLatestResult().getTargets();
    for (var target : targets) {
      if (target.getFiducialId() == 7) {
        return true;
      } else {
        continue;
      }
    }
    return false;
  }
}
