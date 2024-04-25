// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorUtil {

    public static CANSparkMax createSparkMAX(int id, MotorType motorType, int currentLimit) {
        CANSparkMax sparkMAX = new CANSparkMax(id, motorType);
        sparkMAX.restoreFactoryDefaults();
        sparkMAX.setSmartCurrentLimit(currentLimit);

        sparkMAX.burnFlash();

        return sparkMAX;

    }

    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int currentLimit, boolean isInverted,
            boolean isIdleBreak, double slewRate) {
        CANSparkMax sparkMAX = createSparkMAX(id, motortype, currentLimit, isIdleBreak, slewRate);
        sparkMAX.setInverted(isInverted);

        sparkMAX.burnFlash();

        return sparkMAX;
    }

    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int currentLimit, boolean isIdleBreak,
            double slewRate) {
        CANSparkMax sparkMAX = createSparkMAX(id, motortype, currentLimit);

        if (isIdleBreak) {
            sparkMAX.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            sparkMAX.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }

        // built in slew rate for spark max
        sparkMAX.setOpenLoopRampRate(slewRate);

        sparkMAX.burnFlash();

        return sparkMAX;
    }
}
