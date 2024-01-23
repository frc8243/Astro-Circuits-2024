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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.photonvision.EstimatedRobotPose;
import com.fasterxml.jackson.databind.ext.OptionalHandlerFactory;

public class Vision extends SubsystemBase {
  private static PhotonCamera frontCamera;
  private static PhotonPoseEstimator frontCameraPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;

  /** Creates a new Vision. */
  public Vision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    frontCamera = new PhotonCamera("frontCamera");
    frontCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCamera, VisionConstants.kFrontCamtoRobot);
    frontCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/Raspi1 Connected", frontCamera.isConnected());
  }

  public PhotonPipelineResult getFrontCameraResult() {
    return frontCamera.getLatestResult();
  }

  public static Optional<EstimatedRobotPose> getFrontCamEstimatedPose(Pose2d previousEstimatedPose) {
    frontCameraPoseEstimator.setReferencePose(previousEstimatedPose);
    return frontCameraPoseEstimator.update();
  }
}
