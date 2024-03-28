// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.utils.Normalization;

public class TurnToSource extends Command {
  Drivetrain m_drivetrain;
  LEDs m_leds;
  CommandXboxController driverController;
  Alliance ally;
  double targetAngle;
  double currentAngle;

  /** Creates a new TurnToSouce. */
  public TurnToSource(Drivetrain m_drivetrain, LEDs m_leds, CommandXboxController driverController, Alliance ally) {
    this.m_drivetrain = m_drivetrain;
    this.m_leds = m_leds;
    this.driverController = driverController;
    this.ally = ally;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ally == Alliance.Red) {
      targetAngle = ScoringConstants.kRedSourceAngle;
    } else {
      targetAngle = ScoringConstants.kBlueSourceAngle;
    }
    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotSpeed = (targetAngle - currentAngle) * VisionConstants.kRotateP;
    m_drivetrain.drive(
        Normalization.cube(-MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband)),
        Normalization.cube(-MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)),
        rotSpeed,
        true, true);
    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
