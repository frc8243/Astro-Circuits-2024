package frc.robot.subsystems.rollerclaw;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.RobotConstants.MotorConstants;

public class RollerClawReal implements RollerClawIO {
    private static CANSparkMax rollerClawMotor;
    private static RelativeEncoder rollerClawEncoder;

    public RollerClawReal() {
        rollerClawMotor = new CANSparkMax(RollerClawConstants.kRollerClawMotorID, MotorType.kBrushless);
        rollerClawEncoder = rollerClawMotor.getEncoder();
        rollerClawMotor.restoreFactoryDefaults();
        rollerClawMotor.setIdleMode(IdleMode.kBrake);
        rollerClawMotor.setSmartCurrentLimit(MotorConstants.kNEO550CurrentLimit);
        rollerClawMotor.burnFlash();
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
