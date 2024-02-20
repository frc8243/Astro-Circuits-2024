// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ConfigConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ConfigConstants.GyroType;
import frc.robot.commands.GoToTarget;
import frc.robot.commands.TrackTarget;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.DrivetrainSwerve;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroSim;
import frc.robot.subsystems.gyro.NavX;
import frc.robot.subsystems.gyro.Pigeon;
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
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private static Shooter m_shooter;
  private static Vision m_vision;
  private static RollerClaw m_rollerClaw;
  private boolean fieldOrientedDrive = true;
  public LEDs m_leds;
  private static Climber m_climber;
  private static SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    createSubsystems();
    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Robot/PDH", m_pdp);
    SmartDashboard.putData("Drivetrain/Drivetrain", m_drivetrain);
    SmartDashboard.putData("Autos/Selector", m_autoChooser);

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
    driverController.back().onTrue(
        new InstantCommand(() -> fieldOrientedDrive = !fieldOrientedDrive));

    driverController.leftBumper().whileTrue(new TrackTarget(m_vision, m_drivetrain, driverController, m_leds, 7));
    driverController.rightBumper().whileTrue(m_drivetrain.pathFindtoPose(ScoringConstants.kSpeakerCenter));

    operatorController.a().whileTrue(m_shooter.getShooterCommand());
    operatorController.b().whileTrue(m_shooter.getIntakeCommand());
    operatorController.leftBumper().whileTrue(m_rollerClaw.getGrabCommand());
    operatorController.rightBumper().whileTrue(m_rollerClaw.getDumpCommand());
    operatorController.povUp().whileTrue(m_climber.setClimberHeight(0.02));
    operatorController.povDown().whileTrue(m_climber.setClimberHeight(1.3));

    operatorController.leftTrigger(0.1)
        .whileTrue(m_climber.getClimberCommand(-0.25));
    operatorController.rightTrigger(0.1)
        .whileTrue(m_climber.getClimberCommand(0.25));

    operatorController.povLeft().onTrue(new InstantCommand(() -> m_leds.askForNote(1)));
    operatorController.povRight().onTrue(new InstantCommand(() -> m_leds.askForNote(2)));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void createSubsystems() {
    ShooterIO shooterIO;
    DrivetrainIO drivetrainIO;
    RollerClawIO rollerClawIO;
    ClimberIO climberIO;
    GyroIO gyroIO;
    if (ConfigConstants.kRobotGyro == GyroType.Pigeon2) {
      gyroIO = new Pigeon();
    } else {
      gyroIO = new NavX();
    }
    m_gyro = new Gyro(gyroIO);
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterSim();
      drivetrainIO = new DrivetrainSim();
      rollerClawIO = new RollerClawSim();
      climberIO = new ClimberSim();
      gyroIO = new GyroSim();
    } else {
      shooterIO = new ShooterReal();
      drivetrainIO = new DrivetrainSwerve();
      rollerClawIO = new RollerClawReal();
      climberIO = new ClimberReal();

    }
    m_climber = new Climber(climberIO);
    m_shooter = new Shooter(shooterIO);
    m_drivetrain = new Drivetrain(drivetrainIO);
    m_rollerClaw = new RollerClaw(rollerClawIO);
    m_vision = new Vision();
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    m_leds = new LEDs();

  }
}
