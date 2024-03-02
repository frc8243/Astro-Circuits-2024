// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {
  private static ShooterIO shooterIO;
  private DigitalInput shooterSwitch = new DigitalInput(1);
  private static Boolean notePresent = false;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Feed Wheel Speed", shooterIO.getFeedSpeed());
    SmartDashboard.putNumber("Shooter/Shoot Wheel Speed", shooterIO.getShootSpeed());
    SmartDashboard.putBoolean("Shooter/Note Present", notePresent);
    if (shooterSwitch.get()) {
      notePresent = true;

    } else {
      notePresent = false;
    }

    if (Vision.atSpeaker() && notePresent) {
      shooterIO.setShootMotor(ShooterConstants.kShootSpeed);
    } else if (notePresent) {
      shooterIO.setShootMotor(0);
    }

  }

  public Command getShooterCommand() {
    // System.out.println("Shooter shooting");
    return this.startEnd(
        () -> {
          shooterIO.setShootMotor(ShooterConstants.kShootSpeed);
          shooterIO.setFeedMotor(Constants.ShooterConstants.kFeedSpeed);

        },

        () -> {
          shooterIO.stop();
        });
  }

  public static double getShooterSpeed() {
    return shooterIO.getShootSpeed();
  }

  public static boolean getNoteStatus() {
    return notePresent;
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
