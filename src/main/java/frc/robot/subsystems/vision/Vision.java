// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static VisionIO visionIO;

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    visionIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBooleanArray("Vision/CameraStatus", visionIO.checkCams());
  }

  public static Optional<EstimatedRobotPose> getFrontCamPose() {
    return visionIO.getFrontEstPose();
  }

  public static Optional<EstimatedRobotPose> getLeftCamPose() {
    return visionIO.getFrontEstPose();
  }

  public static Optional<EstimatedRobotPose> getRightCamPose() {
    return visionIO.getFrontEstPose();
  }

  public static Matrix<N3, N1> getFrontEstStdDevs(Pose2d estPose) {
    return visionIO.getFrontEstimationStdDevs(estPose);
  }

  public static Matrix<N3, N1> getLeftEstStdDevs(Pose2d estPose) {
    return visionIO.getLeftEstimationStdDevs(estPose);
  }

  public static Matrix<N3, N1> getRightEstStdDevs(Pose2d estPose) {
    return visionIO.getRightEstimationStdDevs(estPose);
  }

  public static boolean getFrontCamConnected() {
    return visionIO.checkCams()[0];
  }

  public static boolean getRightCamConnected() {
    return visionIO.checkCams()[1];
  }

  public static boolean getLeftCamConnected() {
    return visionIO.checkCams()[2];
  }

}
