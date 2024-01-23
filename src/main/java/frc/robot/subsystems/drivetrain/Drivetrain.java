// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class Drivetrain extends SubsystemBase {
  private DrivetrainIO drivetrainIO;

  /** Creates a new Drivetrain. */
  public Drivetrain(DrivetrainIO io) {
    drivetrainIO = io;
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(2.9, 0, 0.1), // Translation
            new PIDConstants(0.975, 0, 0), // Rotation
            AutoConstants.kMaxModuleSpeedMetersPerSecond,
            0.385, // METERS
            new ReplanningConfig()),

        () -> {
          // Basically flips the path for path planner depending on alliance(Origin is
          // Blue Alliance)

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },

        this);
  }

  @Override
  public void periodic() {
    drivetrainIO.updateTelemetry();

  }

  public void drive(double x, double y, double rot, boolean fieldOriented, boolean rateLimit) {
    drivetrainIO.drive(x, y, rot, fieldOriented, rateLimit);
  }

  public void setX() {
    drivetrainIO.setX();
  }

  public Pose2d getPose() {
    return drivetrainIO.getPose();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drivetrainIO.driveRobotRelative(speeds);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drivetrainIO.getRobotRelativeSpeeds();
  }

  public void resetOdometry(Pose2d pose) {
    drivetrainIO.resetOdometry(pose);
  }

  public void resetEncoders() {
    drivetrainIO.resetEncoders();
  }
}
