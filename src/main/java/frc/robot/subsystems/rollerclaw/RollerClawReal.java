package frc.robot.subsystems.rollerclaw;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class RollerClawReal implements RollerClawIO {
    private static CANSparkMax rollerClawMotor;
    private static RelativeEncoder rollerClawEncoder;

    public RollerClawReal() {
        rollerClawMotor = new CANSparkMax(ShooterConstants.kRollerClawMotorID, MotorType.kBrushless);
        rollerClawEncoder = rollerClawMotor.getEncoder();
        rollerClawMotor.restoreFactoryDefaults();
        rollerClawMotor.setIdleMode(IdleMode.kBrake);
        rollerClawMotor.setSmartCurrentLimit(25);
    }

    @Override
    public void setRollerClawMotor(double speed) {
        rollerClawMotor.set(speed);
    }

    @Override
    public double getRollerClawSpeed() {
        return rollerClawEncoder.getVelocity();
    }

    @Override
    public void stop() {
        rollerClawMotor.set(0);
    }
}
