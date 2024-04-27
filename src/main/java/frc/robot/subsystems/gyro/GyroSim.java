package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

public class GyroSim implements GyroIO {
    private Pigeon2 pigeon = new Pigeon2(2, "rio");
    private Pigeon2SimState pigeon2SimState;

    public GyroSim() {
        pigeon2SimState = pigeon.getSimState();
    }

    @Override
    public void resetYaw() {
        pigeon.setYaw(0);
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValue();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch().getValue();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll().getValue();
    }

    @Override
    public double getXAccel() {
        return pigeon.getAccelerationX().getValue();
    }

    @Override
    public double getYAccel() {
        return pigeon.getAccelerationY().getValue();
    }

    @Override
    public double getZAccel() {
        return pigeon.getAccelerationZ().getValue();
    }

    @Override
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

}
