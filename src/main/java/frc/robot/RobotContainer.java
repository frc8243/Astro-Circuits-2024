// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TrackTarget;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.DrivetrainSwerve;
import frc.robot.subsystems.rollerclaw.RollerClaw;
import frc.robot.subsystems.rollerclaw.RollerClawIO;
import frc.robot.subsystems.rollerclaw.RollerClawReal;
import frc.robot.subsystems.rollerclaw.RollerClawSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;

public class RobotContainer {
  private static final RobotContainer m_robotContainer = new RobotContainer();
  private static Drivetrain m_drivetrain;
  private static Gyro m_gyro;
  private static PowerDistribution m_pdp;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private static Shooter m_shooter;
  private static Vision m_vision;
  private static RollerClaw m_rollerClaw;
  private static Field2d m_field;
  private boolean fieldOrientedDrive = true;

  public RobotContainer() {
    createSubsystems();

    SmartDashboard.putData(m_pdp);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_field);

    configureBindings();
    m_drivetrain.setDefaultCommand(new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
            fieldOrientedDrive, true),
        m_drivetrain));
    m_field.setRobotPose(m_drivetrain.getPose());
  }

  private void configureBindings() {
    driverController.x().whileTrue(
        new RunCommand(m_drivetrain::setX));

    driverController.start().onTrue(
        new InstantCommand(m_gyro::resetYaw));
    driverController.back().onTrue(
        new InstantCommand(() -> fieldOrientedDrive = !fieldOrientedDrive));

    driverController.leftBumper().whileTrue(new TrackTarget(m_vision, m_drivetrain, driverController, 8));

    driverController.a().whileTrue(m_shooter.getShooterCommand());
    driverController.b().whileTrue(m_shooter.getIntakeCommand());
    driverController.povUp().whileTrue(m_rollerClaw.getDumpCommand());
    driverController.povDown().whileTrue(m_rollerClaw.getGrabCommand());

  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void createSubsystems() {
    ShooterIO shooterIO;
    DrivetrainIO drivetrainIO;
    RollerClawIO rollerClawIO;
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterSim();
      drivetrainIO = new DrivetrainSim();
      rollerClawIO = new RollerClawSim();

    } else {
      shooterIO = new ShooterReal();
      drivetrainIO = new DrivetrainSwerve();
      rollerClawIO = new RollerClawReal();
    }

    m_shooter = new Shooter(shooterIO);
    m_drivetrain = new Drivetrain(drivetrainIO);
    m_rollerClaw = new RollerClaw(rollerClawIO);
    m_vision = new Vision();
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    m_gyro = new Gyro();
    m_field = new Field2d();
  }
}
