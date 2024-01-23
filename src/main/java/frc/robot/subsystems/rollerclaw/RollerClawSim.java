package frc.robot.subsystems.rollerclaw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ShooterConstants;

public class RollerClawSim implements RollerClawIO {

    private static EncoderSim rollerClawEncoder;

    public RollerClawSim() {
        rollerClawEncoder = new EncoderSim(new Encoder(1, 2));

    }

    @Override
    public void setRollerClawMotor(double speed) {
        // rollerClawMotor.set(speed);
        // throw new UnsupportedOperationException("Unimplemented method
        // 'setRollerClawMotor'");
    }

    @Override
    public double getRollerClawSpeed() {
        // return ((RelativeEncoder) rollerClawEncoder).getVelocity();
        // throw new UnsupportedOperationException("Unimplemented method
        // 'getRollerClawSpeed'");
        return 0;
    }

    @Override
    public void stop() {
        // rollerClawMotor.set(0);
        // throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

}
