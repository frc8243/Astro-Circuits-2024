// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;

/** Add your docs here. */
public class AddrLEDs implements LEDIO {
    private static AddressableLED m_leds = new AddressableLED(0);
    private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kStripLength);
    private static Color idleColor = Color.kBlue;

    public AddrLEDs() {
        m_leds.setLength(LEDConstants.kStripLength);
        m_leds.setData(m_ledBuffer);
        m_leds.start();

    }

    public void setLEDs(Color color) {
        for (int i = 0; i < LEDConstants.kStripLength; i++) {
            m_ledBuffer.setLED(i, color);
        }
    }

    public void blinkLEDs(Color color, double blinkPeriod) {
        boolean blinkOn = ((Timer.getFPGATimestamp() % blinkPeriod) / blinkPeriod) == 0;
        setLEDs(blinkOn ? color : Color.kBlack);
    }

    @Override
    public void periodicLoop() {
        m_leds.setData(m_ledBuffer);
    }

    @Override
    public void noteReady() {
        setLEDs(Color.kGreen);
    }

    @Override
    public void updateIdle(Alliance alliance) {
        if (alliance == Alliance.Red) {
            idleColor = Color.kRed;
        } else {
            idleColor = Color.kBlue;
        }
        setLEDs(idleColor);
    }

    @Override
    public void askForNote(int location) {
        if (location == 1) {
            setLEDs(Color.kYellow);
        } else if (location == 2) {
            setLEDs(Color.kWhite);
        }
    }

    @Override
    public void trackingTag() {
        blinkLEDs(Color.kPink, 0.5);
    }

    @Override
    public void returnToIdle() {
        setLEDs(idleColor);
    }

    @Override
    public void readyToShoot() {
        blinkLEDs(Color.kAzure, 0.25);
    }
}
