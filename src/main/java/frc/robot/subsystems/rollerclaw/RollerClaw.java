// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollerclaw;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class RollerClaw extends SubsystemBase {
  private RollerClawIO rollerClawIO;
  private DigitalInput rollerClawSwitch = new DigitalInput(0);
  private static boolean notePresent = false;

  // Limit switch on DIO 2
  // DigitalInput photoSensor = new DigitalInput(0);

  /** Creates a new RollerClaw. */
  public RollerClaw(RollerClawIO io) {
    this.rollerClawIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RollerClaw/Speed", rollerClawIO.getRollerClawSpeed());
    SmartDashboard.putBoolean("RollerClaw/Note Present", notePresent);
    if (rollerClawSwitch.get()) {
      notePresent = true;
    } else {
      notePresent = false;
    }
  }

  public Command getGrabCommand() {
    return this.startEnd(
        () -> {
          rollerClawIO.setRollerClawMotor(-ShooterConstants.kRollerClawSpeed);
        }, () -> {
          rollerClawIO.stop();
        });

  }

  public Command getDumpCommand() {
    return this.startEnd(
        () -> {
          rollerClawIO.setRollerClawMotor(ShooterConstants.kRollerClawSpeed);
        },

        () -> {
          rollerClawIO.stop();
        });
  }

  public static boolean getNoteStatus() {
    return notePresent;
  }
}
