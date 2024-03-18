// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.leds.LEDIO;

/** Add your docs here. */
public class BlinkinLEDs implements LEDIO {
    private Spark m_blinkin = new Spark(0);
    private Spark m_blinkin2 = new Spark(1);
    private static double color;
    private static double idleColor;

    public BlinkinLEDs() {
        m_blinkin.addFollower(m_blinkin2);
    }

    @Override
    public void periodicLoop() {
        m_blinkin.set(color);
    }

    @Override
    public void noteReady() {
        color = 0.77;
    }

    @Override
    public void updateIdle(Alliance alliance) {
        if (alliance == Alliance.Red) {
            idleColor = 0.61;
        } else {
            idleColor = 0.87;
        }
    }

    @Override
    public void askForNote(int location) {
        if (location == 1) {
            color = 0.65;
        } else if (location == 2) {
            color = 0.93;
        }
    }

    @Override
    public void trackingTag() {
        color = -0.57;
    }

    @Override
    public void returnToIdle() {
        color = idleColor;
    }

    @Override
    public void readyToShoot() {
        color = -0.09;
    }
}
