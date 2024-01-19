package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DrivetrainIO {
    /**
     * Method that drives the robot based on controller inputs
     * 
     * @param x             Speed of robot in X axis (Forwards + Backwards)
     * @param y             Speed of robot in Y axis (Left + Right)
     * @param rot           Speed of robot's rotation.
     * @param fieldOriented Decides if the robot is driving relative to the field or
     *                      to itself
     * @param rateLimit     Rate limiting for smoother control
     */
    public void drive(double x, double y, double rot, boolean fieldOriented, boolean rateLimit);

    public void setX();

    public Pose2d getPose();

    public void driveRobotRelative(ChassisSpeeds speeds);

    public ChassisSpeeds getRobotRelativeSpeeds();

    public void resetOdometry(Pose2d pose);

    public void resetEncoders();

    public void updateTelemetry();
}
