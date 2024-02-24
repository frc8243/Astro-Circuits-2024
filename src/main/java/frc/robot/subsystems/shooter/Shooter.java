// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private DigitalInput shooterSwitch = new DigitalInput(1);
  private Boolean notePresent = false;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Feed Speed", shooterIO.getFeedSpeed());
    SmartDashboard.putNumber("Shooter/Shoot Speed", shooterIO.getShootSpeed());
    SmartDashboard.putBoolean("Shooter/Note Present", notePresent);
    if (shooterSwitch.get()) {
      LEDs.noteReady();
      notePresent = true;

    } else {
      LEDs.returnToIdle();
      notePresent = false;
    }

    if (Vision.atSpeaker() && notePresent) {
      shooterIO.setShootMotor(1);
    } else if (notePresent) {
      shooterIO.setShootMotor(0);
    }

  }

  public Command getShooterCommand() {
    // System.out.println("Shooter shooting");
    return this.startEnd(
        () -> {
          shooterIO.setFeedMotor(Constants.ShooterConstants.kFeedSpeed);

        },

        () -> {
          shooterIO.stop();
        });
  }

  public Command getIntakeCommand() {
    System.out.println("intake intaking");
    return this.startEnd(
        () -> {
          System.out.println("intake intakingInside");
          if (shooterSwitch.get()) {
            shooterIO.stop();
          } else {
            shooterIO.setFeedMotor(-Constants.ShooterConstants.kFeedSpeed);
            shooterIO.setShootMotor(-Constants.ShooterConstants.kShootSpeed);
          }
        },

        () -> {
          shooterIO.stop();
        });
  }

}
