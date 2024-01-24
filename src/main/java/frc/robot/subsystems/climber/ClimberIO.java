package frc.robot.subsystems.climber;

public interface ClimberIO {
    public void setClimbMotor(double speed);

    public void stop();

    public double getClimbSpeed();
}
