// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  private static GyroIO gyroIO;

  /** Creates a new Gyro. */

  public Gyro(GyroIO io) {
    gyroIO = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain/Yaw", getYaw() % 360);
    SmartDashboard.putNumber("Drivetrain/Pitch", getPitch());
    SmartDashboard.putNumber("Drivetrain/Roll", getRoll());

  }

  public void resetYaw() {
    gyroIO.resetYaw();
    System.out.println("Yaw Reset");
  }

  public static double getYaw() {
    return gyroIO.getYaw();
  }

  public static double getPitch() {
    return gyroIO.getPitch();
  }

  public static double getRoll() {
    return gyroIO.getRoll();
  }

  public static double getXAccel() {
    return gyroIO.getXAccel();
  }

  public static double getYAccel() {
    return gyroIO.getYAccel();
  }

}
