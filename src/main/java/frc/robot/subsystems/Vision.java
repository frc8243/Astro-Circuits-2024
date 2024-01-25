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
}
