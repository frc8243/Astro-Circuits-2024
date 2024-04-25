// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;

/** Add your docs here. */
public class DriveCommands {
    private Drivetrain m_drivetrain;
    private static LEDs m_leds;
    private CommandXboxController driverController;
    private static Alliance ally;

    public DriveCommands(Drivetrain m_drivetrain, LEDs m_leds, CommandXboxController driverController, Alliance ally) {
        this.m_drivetrain = m_drivetrain;
        this.m_leds = m_leds;
        this.driverController = driverController;
        this.ally = ally;
    }

    public static final Command TurnToSource() {
        return new FunctionalCommand(
            ()->{
            m_leds.trackingTarget();
            double targetAngle = (ally.equals(Alliance.Red) ? ); 
            }, 
        null, null, null, null);
}
