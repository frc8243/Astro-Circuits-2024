package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final int kClimbMotorID = 10;
    public static final double kpPos = 10;
    public static final double kiPos = 0.0;
    public static final double kdPos = 0.5;
    public static final double kFeedForward = 0.617753;
    public static final double kGravityCompensation = 0.059;
    public static final double kMaxVel = 1;
    public static final double kMaxAccel = 2.50;
    public static final double kSimMeasurementStdDev = 0.0;
    public static final double kMinClimberHeight = 0.0;
    public static final double kMaxClimberHeight = 0.75;
    public static final double kMetersPerRevolution = Units.inchesToMeters(27) / 238.77;
    public static final double kHeightTolerance = Units.inchesToMeters(1);
    public static final double kVelocityTolerance = kMaxVel / 50.0;
    public static final double kClimberGearRatio = 48.0 / 1.0;
}
