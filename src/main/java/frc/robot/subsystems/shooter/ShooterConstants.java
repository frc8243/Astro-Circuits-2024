package frc.robot.subsystems.shooter;

import frc.robot.RobotConstants.MotorConstants;

public class ShooterConstants {
    public static final int kShootMotorID = 2;
    public static final int kFeedMotorID = 4;
    public static final int kRollerClawMotorID = 3;
    public static final double kFeedSpeed = 1;
    public static final double kShootSpeed = 1;
    public static final double kShooterReadySpeed = 5900;
    public static final double kP = 0.00025;
    public static final double kI = 0;
    public static final double kD = 0.002;
    public static final double kFF = 1 / MotorConstants.kNEOFreeSpeed;
    public static final double kRPMTolerance = 150;
}
