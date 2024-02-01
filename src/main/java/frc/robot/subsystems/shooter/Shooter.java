// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Feed Speed", shooterIO.getFeedSpeed());
    SmartDashboard.putNumber("Shooter/Shoot Speed", shooterIO.getShootSpeed());
  }

  public Command getShooterCommand() {
    System.out.println("Shooter shooting");
    return this.startEnd(
        () -> {
          System.out.println("Shooter shootingInside");
          shooterIO.setShootMotor(Constants.ShooterConstants.kShootSpeed);

          try {
            Thread.sleep(500);
          } catch (InterruptedException e) {

            e.printStackTrace();
          }
          System.out.println("Past the wait point");
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
          shooterIO.setFeedMotor(-Constants.ShooterConstants.kFeedSpeed);
          shooterIO.setShootMotor(-Constants.ShooterConstants.kShootSpeed);

        },

        () -> {
          shooterIO.stop();
        });
  }

}
