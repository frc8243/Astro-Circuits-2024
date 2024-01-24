// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class GoToTarget extends Command {
  private Vision m_vision;
  private Drivetrain m_drivetrain;
  private CommandXboxController driverController;
  private int targetTag;
  private double targetDistance;
  private double currentDistance;

  /** Creates a new GoToTarget. */
  public GoToTarget(Vision vision, Drivetrain drivetrain, CommandXboxController controller, int targetTag,
      double targetDistance) {
    this.m_vision = vision;
    this.m_drivetrain = drivetrain;
    this.driverController = controller;
    this.targetTag = targetTag;
    this.targetDistance = targetDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision, m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = m_vision.getFrontCameraResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == targetTag) {
          double xSpeed = target.getBestCameraToTarget().getX() * VisionConstants.kTranslateP;
          double ySpeed = target.getBestCameraToTarget().getY() * VisionConstants.kTranslateP;
          double rotSpeed = -(target.getYaw()) * VisionConstants.kTurningP;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentDistance <= targetDistance;
  }
}
