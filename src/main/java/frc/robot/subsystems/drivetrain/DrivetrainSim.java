package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainSim implements DrivetrainIO {

    @Override
    public void drive(double x, double y, double rot, boolean fieldOriented, boolean rateLimit) {
    }

    @Override
    public void setX() {
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d();
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return null;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
    }

    @Override
    public void resetEncoders() {
    }

    @Override
    public void updateTelemetry() {

    }

}
