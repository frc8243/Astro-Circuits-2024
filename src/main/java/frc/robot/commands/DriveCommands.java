// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision.VisionConstants;
import frc.utils.Normalization;
import frc.robot.commands.CommandConstants;

/** Add your docs here. */
@SuppressWarnings("unused")
public class DriveCommands {
    private static Drivetrain m_drivetrain;
    private static LEDs m_leds;
    private static CommandXboxController driverController;
    private static Alliance ally;
    private static double targetAngle;
    private static double currentAngle;

    public DriveCommands(Drivetrain m_drivetrain, LEDs m_leds, CommandXboxController driverController, Alliance ally) {
        DriveCommands.m_drivetrain = m_drivetrain;
        DriveCommands.m_leds = m_leds;
        DriveCommands.driverController = driverController;
        DriveCommands.ally = ally;
    }

    public void setAlliance(Alliance ally) {
        DriveCommands.ally = ally;
    }

    public static final Command TurnToSource() {
        return new FunctionalCommand(
                () -> {
                    SmartDashboard.putBoolean("DriverAssists/TurningToSource", true);
                    m_leds.trackingTarget();
                    targetAngle = (ally.equals(Alliance.Red) ? FieldConstants.kRedSourceAngle
                            : FieldConstants.kBlueSourceAngle);
                    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
                },
                () -> {
                    double rotSpeed = (targetAngle - currentAngle) * CommandConstants.kRotateP;
                    m_drivetrain.drive(
                            Normalization.cube(
                                    -MathUtil.applyDeadband(driverController.getLeftY(),
                                            RobotConstants.kDriveDeadband)),
                            Normalization.cube(
                                    -MathUtil.applyDeadband(driverController.getLeftX(),
                                            RobotConstants.kDriveDeadband)),
                            rotSpeed,
                            true);
                    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
                    if (MathUtil.isNear(targetAngle, currentAngle, CommandConstants.kSourceAlignmentTolerance)) {
                        m_leds.linedUp();
                    }
                },
                interrupted -> {
                    m_leds.returnToIdle();
                    SmartDashboard.putBoolean("DriverAssists/TurningToSource", false);
                },
                () -> MathUtil.isNear(targetAngle, currentAngle, CommandConstants.kSourceAlignmentTolerance),
                m_drivetrain);
    }

    public static final Command TurnToSpeaker() {
        return new FunctionalCommand(
                () -> {
                    SmartDashboard.putBoolean("DriverAssists/TurningToSpeaker", true);
                    m_leds.trackingTarget();
                    targetAngle = 0;
                    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
                },
                () -> {
                    double rotSpeed = (targetAngle - currentAngle) * CommandConstants.kRotateP;
                    m_drivetrain.drive(
                            Normalization.cube(
                                    -MathUtil.applyDeadband(driverController.getLeftY(),
                                            RobotConstants.kDriveDeadband)),
                            Normalization.cube(
                                    -MathUtil.applyDeadband(driverController.getLeftX(),
                                            RobotConstants.kDriveDeadband)),
                            rotSpeed,
                            true);
                    currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
                    if (MathUtil.isNear(targetAngle, currentAngle, CommandConstants.kSourceAlignmentTolerance)) {
                        m_leds.linedUp();
                    }
                },
                interrupted -> {
                    m_leds.returnToIdle();
                    SmartDashboard.putBoolean("DriverAssists/TurningToSource", true);
                },
                () -> MathUtil.isNear(targetAngle, currentAngle, CommandConstants.kSourceAlignmentTolerance),
                m_drivetrain);
    }

}
