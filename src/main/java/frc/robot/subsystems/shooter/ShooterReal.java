package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ShooterReal implements ShooterIO {
    private static CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShootMotorID, MotorType.kBrushless);
    private static CANSparkMax feedMotor = new CANSparkMax(ShooterConstants.kFeedMotorID, MotorType.kBrushless);
    private static RelativeEncoder shootEncoder = shootMotor.getEncoder();
    private static RelativeEncoder feedEncoder = feedMotor.getEncoder();

    public ShooterReal() {

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
    public double getFeedSpeed() {
        return feedEncoder.getVelocity();
    }

    @Override
    public double getShootSpeed() {
        return shootEncoder.getVelocity();
    }
}
