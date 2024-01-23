package frc.robot.subsystems.climber;

public interface ClimberIO {
    public void setFeedMotor(double speed);

    public void setShootMotor(double speed);

    public void stop();

    public double getFeedSpeed();

    public double getShootSpeed();
}
