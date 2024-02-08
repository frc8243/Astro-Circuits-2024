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

  // Limit switch on DIO 2
  // DigitalInput photoSensor = new DigitalInput(0);

  /** Creates a new RollerClaw. */
  public RollerClaw(RollerClawIO io) {
    this.rollerClawIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RollerClaw/Speed", rollerClawIO.getRollerClawSpeed());
    // if (photoSensor.get() == true) {
    // System.out.println("Sensor Sensing Periodically");
    // }
    // This method will be called once per scheduler run
  }

  public Command getGrabCommand() {
    System.out.println("Grabbing Note");
    return this.run(
        () -> {
          // Runs the motor forwards while the photosensor sees nothing, stops otherwise
          // if (photoSensor.get() == true) {
          rollerClawIO.setRollerClawMotor(-ShooterConstants.kRollerClawSpeed);
          System.out.println("Sensor sensing");
          // } else {
          rollerClawIO.stop();
          // }

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
