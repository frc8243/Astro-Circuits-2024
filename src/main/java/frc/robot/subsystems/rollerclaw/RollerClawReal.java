package frc.robot.subsystems.rollerclaw;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.RobotConstants.MotorConstants;
import frc.utils.MotorUtil;

public class RollerClawReal implements RollerClawIO {
    private static CANSparkMax rollerClawMotor;
    private static RelativeEncoder rollerClawEncoder;

    public RollerClawReal() {
        rollerClawMotor = MotorUtil.createSparkMAX(RollerClawConstants.kRollerClawMotorID, MotorType.kBrushless,
                MotorConstants.kNEO550CurrentLimit, true);
        rollerClawEncoder = rollerClawMotor.getEncoder();
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
