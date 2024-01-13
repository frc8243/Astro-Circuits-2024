// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
public class RobotContainer {
  private static final RobotContainer m_RobotContainer = new RobotContainer();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private static Shooter m_shooter;
  public RobotContainer() {
    createSubsystems();
    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(m_shooter.getShooterCommand());
  }

  public static RobotContainer getInstance() {
    return m_RobotContainer;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void createSubsystems() {
    ShooterIO shooterIO;
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterSim();
    }
    else {
      shooterIO = new ShooterReal();
    }

    m_shooter = new Shooter(shooterIO);
  }
}
