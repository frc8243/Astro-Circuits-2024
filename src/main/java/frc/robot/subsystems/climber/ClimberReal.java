package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.NeoMotorConstants;
import frc.utils.MotorUtil;

public class ClimberReal implements ClimberIO {

    public static CANSparkMax climberMotorController;
    public static RelativeEncoder climberEncoder;

    public ClimberReal() {
        climberMotorController = MotorUtil.createSparkMAX(ClimberConstants.kClimbMotorID, MotorType.kBrushless,
                NeoMotorConstants.kNeoCurrentLimit, false, true, 0.1);

        climberEncoder = climberMotorController.getEncoder();
        climberEncoder.setPositionConversionFactor(ClimberConstants.kMetersPerRevolution);
        // dividng by 60 to convert meters per miniute to meters per seconds
        climberEncoder.setVelocityConversionFactor(ClimberConstants.kMetersPerRevolution / 60);
    }

    @Override
    public double getEncoderPosition() {
        return climberEncoder.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return climberEncoder.getVelocity();
    }

    @Override
    public void setMotorSpeed(double speed) {
        climberMotorController.set(speed);
    }

    @Override
    public void setEncoderPosition(double position) {
        climberEncoder.setPosition(position);
    }

    @Override
    public double getClimberCurrent() {
        return climberMotorController.getOutputCurrent();
    }

    @Override
    public void periodicUpdate() {
        // Only code in here that relates a physical subsystem
        SmartDashboard.putNumber("Climber/Real motor temp (C)", climberMotorController.getMotorTemperature());
        SmartDashboard.putNumber("Climber/Motor Speed (-1 <-> 1)", climberMotorController.get());
    }

}