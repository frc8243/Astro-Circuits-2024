// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  private final static Pigeon2 gyro = new Pigeon2(2, "rio");

  /** Creates a new Gyro. */

  public Gyro() {
    gyro.setYaw(0);
    gyro.getYaw().setUpdateFrequency(100);
    gyro.getGravityVectorZ().setUpdateFrequency(100);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain/Yaw", getYaw());
    SmartDashboard.putNumber("Drivetrain/Pitch", getPitch());
    SmartDashboard.putNumber("Drivetrain/Roll", getRoll());

  }

  public void resetYaw() {
    gyro.setYaw(0);
    System.out.println("Yaw Reset");
  }

  public static double getYaw() {
    return gyro.getYaw().getValue();
  }

  public static double getPitch() {
    return gyro.getPitch().getValue();
  }

  public static double getRoll() {
    return gyro.getRoll().getValue();
  }

  public static double getXAccel() {
    return gyro.getAccelerationX().getValue();
  }

  public static double getYAccel() {
    return gyro.getAccelerationY().getValue();

  }

  public static double getZAccel() {
    return gyro.getAccelerationZ().getValue();
  }

}
