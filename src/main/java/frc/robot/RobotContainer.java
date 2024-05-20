// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.AddrLEDs;
import frc.robot.subsystems.leds.BlinkinLEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleReal;
import frc.robot.subsystems.drivetrain.SwerveModuleSim;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.DriveConstants;
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
import frc.robot.subsystems.shooter.ShooterKraken;
import frc.robot.subsystems.shooter.ShooterNEO;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.utils.Normalization;
import frc.robot.RobotConstants.*;

public class RobotContainer {
  private static final RobotContainer m_robotContainer = new RobotContainer();
  private static Drivetrain m_drivetrain;
  private static Gyro m_gyro;
  private static PowerDistribution m_pdp;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private static Shooter m_shooter;
  public static Vision m_vision;
  private static RollerClaw m_rollerClaw;
  private boolean fieldOrientedDrive = true;
  public static LEDs m_leds;
  private static Climber m_climber;
  private static SendableChooser<Command> m_autoChooser;
  public static Alliance m_alliance;
  private static DriveCommands m_driveCommands;

  public RobotContainer() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      m_alliance = ally.get();
    } else {
      m_alliance = Alliance.Blue;
    }

    createSubsystems();

    NamedCommands.registerCommand("shoot", m_shooter.getAdvancedShooterCommand().withTimeout(5));
    NamedCommands.registerCommand("dump", m_rollerClaw.getDumpCommand());

    configureBindings();

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Robot/PDH", m_pdp);
    SmartDashboard.putData("Drivetrain/Drivetrain", m_drivetrain);
    SmartDashboard.putData("Autos/Selector", m_autoChooser);
    SmartDashboard.putString("Robot/Alliance", m_alliance.toString());

    m_drivetrain.setDefaultCommand(new RunCommand(
        () -> m_drivetrain.drive(
            Normalization.cube(-MathUtil.applyDeadband(driverController.getLeftY(), RobotConstants.kDriveDeadband)),
            Normalization.cube(-MathUtil.applyDeadband(driverController.getLeftX(), RobotConstants.kDriveDeadband)),
            Normalization.cube(-MathUtil.applyDeadband(driverController.getRightX(), RobotConstants.kDriveDeadband)),
            fieldOrientedDrive),
        m_drivetrain));

  }

  private void configureBindings() {
    driverController.x().whileTrue(new RunCommand(m_drivetrain::setX));

    driverController.start().onTrue(new InstantCommand(m_gyro::resetYaw));
    driverController.back().onTrue(new InstantCommand(() -> fieldOrientedDrive = !fieldOrientedDrive));

    driverController.leftBumper().onTrue(DriveCommands.TurnToSpeaker());
    driverController.rightBumper().onTrue(DriveCommands.TurnToSource());

    operatorController.a().whileTrue(m_shooter.getAdvancedShooterCommand());
    operatorController.b().whileTrue(m_shooter.getIntakeCommand());
    operatorController.leftBumper().whileTrue(m_rollerClaw.getGrabCommand());
    operatorController.rightBumper().whileTrue(m_rollerClaw.getDumpCommand());
    operatorController.povUp().onTrue(m_climber.setClimberHeight(-0.65));
    operatorController.povDown().onTrue(m_climber.setClimberHeight(0));
    operatorController.leftTrigger(0.1).whileTrue(m_climber.getClimberCommand(-0.25));
    operatorController.rightTrigger(0.1).whileTrue(m_climber.getClimberCommand(0.25));

    operatorController.povLeft().onTrue(new InstantCommand(() -> m_leds.askForNote(1)));
    operatorController.povRight().onTrue(new InstantCommand(() -> m_leds.askForNote(2)));

    driverController.povUp().onTrue(m_shooter.playSong());
    driverController.povDown().onTrue(m_shooter.pause());
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void createSubsystems() {
    ShooterIO shooterIO;
    SwerveModuleIO frontLeftModuleIO;
    SwerveModuleIO frontRightModuleIO;
    SwerveModuleIO rearLeftModuleIO;
    SwerveModuleIO rearRightModuleIO;
    RollerClawIO rollerClawIO;
    ClimberIO climberIO;
    GyroIO gyroIO;
    LEDIO ledIO;
    VisionIO visionIO;
    if (RobotConstants.kRobotGyro == GyroType.Pigeon2) {
      gyroIO = new Pigeon();
    } else {
      gyroIO = new NavX();
    }
    m_gyro = new Gyro(gyroIO);
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterSim();
      rollerClawIO = new RollerClawSim();
      climberIO = new ClimberSim();
      gyroIO = new GyroSim();
      visionIO = new VisionSim();
      frontLeftModuleIO = new SwerveModuleSim("frontLeft");
      frontRightModuleIO = new SwerveModuleSim("frontRight");
      rearLeftModuleIO = new SwerveModuleSim("rearLeft");
      rearRightModuleIO = new SwerveModuleSim("rearRight");
    } else {
      if (RobotConstants.kShooterMotors == ShooterMotorType.Krakens) {
        shooterIO = new ShooterKraken();
      } else {
        shooterIO = new ShooterNEO();
      }
      rollerClawIO = new RollerClawReal();
      climberIO = new ClimberReal();
      visionIO = new VisionReal();
      frontLeftModuleIO = new SwerveModuleReal(DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset, "frontLeft");
      frontRightModuleIO = new SwerveModuleReal(DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset, "frontRight");
      rearLeftModuleIO = new SwerveModuleReal(DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId, DriveConstants.kRearLeftChassisAngularOffset, "rearLeft");
      rearRightModuleIO = new SwerveModuleReal(DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId, DriveConstants.kRearRightChassisAngularOffset, "rearRight");

    }
    if (RobotConstants.kRobotLEDs == LEDType.Blinkin) {
      ledIO = new BlinkinLEDs();
    } else {
      ledIO = new AddrLEDs();
    }
    m_climber = new Climber(climberIO);
    m_shooter = new Shooter(shooterIO);
    m_drivetrain = new Drivetrain(new SwerveModule(frontLeftModuleIO), new SwerveModule(frontRightModuleIO),
        new SwerveModule(rearLeftModuleIO), new SwerveModule(rearRightModuleIO));
    m_rollerClaw = new RollerClaw(rollerClawIO);
    m_vision = new Vision(visionIO);
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    m_leds = new LEDs(ledIO);

    m_driveCommands = new DriveCommands(m_drivetrain, m_leds, driverController, m_alliance);

  }

  public void setAlliance(Alliance ally) {
    m_alliance = ally;
    m_leds.updateIdle(ally);
    m_drivetrain.setAlliance(ally);
    m_driveCommands.setAlliance(ally);
  }

  public static Alliance getAlliance() {
    return m_alliance;
  }
}