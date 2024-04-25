package frc.robot;

public class RobotConstants {
    enum GyroType {
        NavX,
        Pigeon2,
    }

    enum LEDType {
        Blinkin,
        Addressable,
    }

    enum ShooterMotorType {
        Krakens,
        NEOs
    }

    // public static final GyroType kRobotGyro = GyroType.NavX;
    public static final GyroType kRobotGyro = GyroType.Pigeon2;
    public static final LEDType kRobotLEDs = LEDType.Addressable;
    public static final ShooterMotorType kShooterMotors = ShooterMotorType.Krakens;
    // public static final LEDType kRobotLEDs = LEDType.Blinkin;
    // public static final ShooterMotorType = ShooterMotorType.NEOs;

    /**
     * Max robot speed in meters per second (m/s)
     */
    public static final double kMaxSpeed = 4.75;

    /**
     * Max robot rotational speed in radians per second (rad/s)
     * 
     */
    // TODO: Convert this to degrees, and wrap in a `Units.degreesToRadians()' since
    // we don't think in radians!
    public static final double kMaxRotationSpeed = 2 * Math.PI; // Radians per Second

    public static final double kDriveDeadband = 0.1;

    public static final class MotorConstants {
        /* Suggested current limit for various motors, in amps (A) */
        public static final int kNEOCurrentLimit = 60;
        public static final int kKrakenCurrentLimit = 80;
        public static final int kNEO550CurrentLimit = 20;

        /* Free speeds of various motors, in rotations per minute (rpm) */
        public static final int kNEOFreeSpeed = 5676;
    }
}
