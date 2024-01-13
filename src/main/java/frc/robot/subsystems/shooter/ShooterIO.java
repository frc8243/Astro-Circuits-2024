package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void setFeedMotor(double speed);
    public void setShootMotor(double speed);
    public void stop();
    public double getFeedSpeed();
    public double getShootSpeed();
}
