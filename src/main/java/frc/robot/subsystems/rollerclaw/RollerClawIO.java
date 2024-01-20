package frc.robot.subsystems.rollerclaw;

public interface RollerClawIO {
    public void setRollerClawMotor(double speed);

    public double getRollerClawSpeed();

    public void stop();
}
