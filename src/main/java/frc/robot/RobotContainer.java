// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.DrivetrainSwerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;

public class RobotContainer {
  private static final RobotContainer m_RobotContainer = new RobotContainer();
  private static Drivetrain m_drivetrain;
  private static Gyro m_gyro;
  private static PowerDistribution m_pdp;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private static Shooter m_shooter;
  private static Vision m_vision;
  private boolean fieldOrientedDrive = true;

  public RobotContainer() {
    m_gyro = new Gyro();
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    createSubsystems();
    m_vision = new Vision(m_drivetrain);

    SmartDashboard.putData(m_pdp);
    SmartDashboard.putData(m_drivetrain);

    configureBindings();
    m_drivetrain.setDefaultCommand(new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
            fieldOrientedDrive, true),
        m_drivetrain));
  }

  private void configureBindings() {
    driverController.x().whileTrue(
        new RunCommand(m_drivetrain::setX));

    driverController.start().onTrue(
        new InstantCommand(m_gyro::resetYaw));
    driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_vision.turnToTarget(1)));

    driverController.a().whileTrue(m_shooter.getShooterCommand());
    driverController.b().whileTrue(m_shooter.getIntakeCommand());

  }

  public static RobotContainer getInstance() {
    return m_RobotContainer;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void createSubsystems() {
    ShooterIO shooterIO;
    DrivetrainIO drivetrainIO;
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterSim();
      drivetrainIO = new DrivetrainSim();

    } else {
      shooterIO = new ShooterReal();
      drivetrainIO = new DrivetrainSwerve();
    }

    m_shooter = new Shooter(shooterIO);
    m_drivetrain = new Drivetrain(drivetrainIO);
  }
}
