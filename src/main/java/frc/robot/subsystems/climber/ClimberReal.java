package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ClimberReal implements ClimberIO {
    private static CANSparkMax climbMotor = new CANSparkMax(ClimberConstants.kClimbMotorID, MotorType.kBrushless);
    private static RelativeEncoder climbEncoder = climbMotor.getEncoder();

    public ClimberReal() {

    }

    @Override
    public void setClimbMotor(double speed) {
        climbMotor.set(speed);
    }

    @Override
    public void stop() {
        climbMotor.set(0);
    }

    @Override
    public double getClimbSpeed() {
        return climbEncoder.getVelocity();
    }
}
