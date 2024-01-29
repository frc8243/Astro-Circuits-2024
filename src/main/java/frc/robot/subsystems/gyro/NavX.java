// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class NavX implements GyroIO {
    private static AHRS navx = new AHRS(SPI.Port.kMXP);

    @Override
    public void resetYaw() {
        navx.reset();
    }

    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public double getRoll() {
        return navx.getRoll();
    }

    @Override
    public double getXAccel() {
        return navx.getRawAccelX();
    }

    @Override
    public double getYAccel() {
        return navx.getRawAccelY();
    }

    @Override
    public double getZAccel() {
        return navx.getRawAccelZ();
    }

}
