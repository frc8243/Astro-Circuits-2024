// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollerclaw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;

public class RollerClaw extends SubsystemBase {
  private RollerClawIO rollerClawIO;

  /** Creates a new RollerClaw. */
  public RollerClaw(RollerClawIO io) {
    this.rollerClawIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RollerClaw/Speed", rollerClawIO.getRollerClawSpeed());
    // This method will be called once per scheduler run
  }

  public Command getGrabCommand() {
    System.out.println("Grabbing Note");
    return this.startEnd(
        () -> {
          rollerClawIO.setRollerClawMotor(-ShooterConstants.kRollerClawSpeed);
        },

        () -> {
          rollerClawIO.stop();
        });

  }

  public Command getDumpCommand() {
    System.out.println("Dumping Note");
    return this.startEnd(
        () -> {
          rollerClawIO.setRollerClawMotor(ShooterConstants.kRollerClawSpeed);
        },

        () -> {
          rollerClawIO.stop();
        });
  }
}
