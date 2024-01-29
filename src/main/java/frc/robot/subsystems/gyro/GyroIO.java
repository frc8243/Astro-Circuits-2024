package frc.robot.subsystems.gyro;

public interface GyroIO {

    public void resetYaw();

    public double getYaw();

    public double getPitch();

    public double getRoll();

    public double getXAccel();

    public double getYAccel();

    public double getZAccel();

}
