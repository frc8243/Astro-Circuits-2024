// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Vision extends SubsystemBase {
  private PhotonCamera frontCamera;
  private Drivetrain m_drivetrain;

  /** Creates a new Vision. */
  public Vision(Drivetrain drivetrain) {
    frontCamera = new PhotonCamera("frontCamera");
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/Raspi1 Connected", frontCamera.isConnected());
  }

  public void turnToTarget(int target) {
    var result = frontCamera.getLatestResult();
    if (result.hasTargets()) {
      if (result.getBestTarget().getFiducialId() == target) {
        double rotationSpeed = -(result.getBestTarget().getYaw()) * VisionConstants.kTurningP;
        m_drivetrain.drive(0, 0, rotationSpeed, false, false);
      }
    }

  }
}
