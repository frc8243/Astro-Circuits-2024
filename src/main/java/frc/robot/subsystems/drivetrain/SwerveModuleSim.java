package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.ModuleConstants;
import frc.utils.SimEncoder;

public class SwerveModuleSim implements SwerveModuleIO {
    private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDrivingMotorReduction, 0.025);
    private DCMotorSim turnMotor = new DCMotorSim(DCMotor.getNeo550(1), ModuleConstants.kTurningMotorReduction, 0.025);

    private SimEncoder m_drivingEncoder;
    private SimEncoder m_turningEncoder;

    private PIDController m_turningPIDController = new PIDController(10, 0.0, 0.0);
    private PIDController m_drivingPIDController = new PIDController(1, 0.0, 0.0);

    private double driveMotorOutput = 0.0;
    private double turningMotorOutput = 0.0;

    private double chassisAngularOffset;

    private String name;

    public SwerveModuleSim(String name) {
        m_drivingEncoder = new SimEncoder(name + " DriveEconder");
        m_turningEncoder = new SimEncoder(name + " TurnEconder");
        this.name = name;
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveMotor.update(0.02);
        turnMotor.update(0.02);
        m_drivingEncoder.setDistance(driveMotor.getAngularPositionRotations());
        m_drivingEncoder.setSpeed(driveMotor.getAngularVelocityRadPerSec());
        m_turningEncoder.setDistance(turnMotor.getAngularPositionRad());
    };

    public void setDriveEncoderPosition(double position) {
        m_drivingEncoder.setDistance(position);
    };

    public double getDriveEncoderPosition() {
        return m_drivingEncoder.getDistance();
    };

    public double getDriveEncoderSpeedMPS() {
        return m_drivingEncoder.getSpeed();
    };

    public double getTurnEncoderPosition() {
        return m_turningEncoder.getDistance();
    };

    public void setDesiredDriveSpeedMPS(double speed) {
        driveMotorOutput = m_drivingPIDController.calculate(getDriveEncoderSpeedMPS(), speed);

        // Apply PID output
        driveMotor.setInputVoltage(driveMotorOutput);
        driveMotor.getAngularVelocityRPM();

    };

    public void setDesiredTurnAngle(double angle) {
        turningMotorOutput = m_turningPIDController.calculate(getTurnEncoderPosition(), angle);

        // Apply PID output
        turnMotor.setInputVoltage(turningMotorOutput);
    };

    public double getDriveBusVoltage() {
        return 0;
    };

    public double getDriveOutput() {
        return driveMotorOutput;
    }

    public double getTurnBusVoltage() {
        return 0;
    }

    public double getTurnOutput() {
        return turningMotorOutput;
    }

    public String getName() {
        return name;
    }

    public double getChassisAngularOffset() {
        return 0.0;
    }
}
