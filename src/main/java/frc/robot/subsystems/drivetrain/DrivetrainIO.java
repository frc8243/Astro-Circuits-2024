package frc.robot.subsystems.drivetrain;

public interface DrivetrainIO {
    public void drive(double x, double y, double rot, boolean fieldOriented, boolean rateLimit);
}
