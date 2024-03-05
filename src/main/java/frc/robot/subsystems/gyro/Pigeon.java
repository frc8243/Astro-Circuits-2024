// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Add your docs here. */
public class Pigeon implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(2, "rio");

    @Override
    public void resetYaw() {
        pigeon.setYaw(0);
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValue();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch().getValue();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll().getValue();
    }

    @Override
    public double getXAccel() {
        return pigeon.getAccelerationX().getValue();
    }

    @Override
    public double getYAccel() {
        return pigeon.getAccelerationY().getValue();
    }

    @Override
    public double getZAccel() {
        return pigeon.getAccelerationZ().getValue();
    }

}
