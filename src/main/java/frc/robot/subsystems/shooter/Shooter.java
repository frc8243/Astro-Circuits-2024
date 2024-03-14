// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {
  private static ShooterIO shooterIO;
  private DigitalInput shooterSwitch = new DigitalInput(1);
  private static Boolean notePresent = false;
  private Timer timer = new Timer();
  private double targetRPM;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
    targetRPM = NeoMotorConstants.kFreeSpeedRpm;
  }

  @Override
  public void periodic() {
    if (timer.hasElapsed(30)) {
      targetRPM = NeoMotorConstants.kFreeSpeedRpm * (RobotController.getBatteryVoltage() / 12);
    }
    SmartDashboard.putNumber("Shooter/Feed Wheel Speed", shooterIO.getFeedSpeed());
    SmartDashboard.putNumber("Shooter/Shoot Wheel Speed", shooterIO.getShootSpeed());
    SmartDashboard.putBoolean("Shooter/Note Present", notePresent);
    SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
    if (shooterSwitch.get()) {
      notePresent = true;

    } else {
      notePresent = false;
    }

    if (Vision.atSpeaker() && notePresent) {
      shooterIO.setShootMotor(ShooterConstants.kShootSpeed);
    } else if (notePresent) {
      shooterIO.stop();
    }

  }

  public Command getAdvancedShooterCommand() {
    return this.startEnd(
        () -> {
          shooterIO.spinShootMotor(targetRPM);
          if (MathUtil.isNear(targetRPM, shooterIO.getShootSpeed(), ShooterConstants.kRPMTolerance) == true) {
            shooterIO.spinFeedMotor(targetRPM);
          }
        },
        () -> {
          shooterIO.stop();
        });
  }

  public Command getAdvancedIntakeCommand() {
    return this.startEnd(
        () -> {
          shooterIO.spinShootMotor(-targetRPM);
          shooterIO.spinFeedMotor(-targetRPM);
        },
        () -> {
          shooterIO.stop();
        });
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
