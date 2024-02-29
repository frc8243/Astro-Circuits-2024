package frc.robot.subsystems.climber;

public interface ClimberIO {

    public double getEncoderPosition();

    public double getEncoderSpeed();

    public void setMotorSpeed(double speed);

    public void setEncoderPosition(double position);

    public void periodicUpdate();

    public double getClimberCurrent();

}
