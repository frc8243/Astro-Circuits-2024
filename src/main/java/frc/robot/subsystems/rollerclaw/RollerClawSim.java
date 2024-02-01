package frc.robot.subsystems.rollerclaw;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class RollerClawSim implements RollerClawIO {

    public EncoderSim rollerClawEncoder;

    public RollerClawSim() {
        rollerClawEncoder = new EncoderSim(new Encoder(2, 3));

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
