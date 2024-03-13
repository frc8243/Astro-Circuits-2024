package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ShooterReal implements ShooterIO {
    private static CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShootMotorID, MotorType.kBrushless);
    private static SparkPIDController shootController = shootMotor.getPIDController();
    private static CANSparkMax feedMotor = new CANSparkMax(ShooterConstants.kFeedMotorID, MotorType.kBrushless);
    private static SparkPIDController feedController = feedMotor.getPIDController();
    private static RelativeEncoder shootEncoder = shootMotor.getEncoder();
    private static RelativeEncoder feedEncoder = feedMotor.getEncoder();

    public ShooterReal() {
        shootMotor.restoreFactoryDefaults();
        feedMotor.restoreFactoryDefaults();
        shootMotor.setIdleMode(IdleMode.kCoast);
        feedMotor.setIdleMode(IdleMode.kCoast);
        shootMotor.setSmartCurrentLimit(NeoMotorConstants.kNeoCurrentLimit);
        feedMotor.setSmartCurrentLimit(NeoMotorConstants.kNeoCurrentLimit);

        shootController.setP(ShooterConstants.kP);
        shootController.setI(ShooterConstants.kI);
        shootController.setD(ShooterConstants.kD);

        feedController.setP(ShooterConstants.kP);
        feedController.setI(ShooterConstants.kI);
        feedController.setD(ShooterConstants.kD);

    }

    @Override
    public void setFeedMotor(double speed) {
        feedMotor.set(speed);
    }

    @Override
    public void setShootMotor(double speed) {
        shootMotor.set(speed);
    }

    @Override
    public void stop() {
        shootMotor.set(0);
        feedMotor.set(0);
    }

    @Override
    public void spinShootMotor(double rpm) {
        shootController.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public void spinFeedMotor(double rpm) {
        feedController.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public double getFeedSpeed() {
        return feedEncoder.getVelocity();
    }

    @Override
    public double getShootSpeed() {
        return shootEncoder.getVelocity();
    }

}
