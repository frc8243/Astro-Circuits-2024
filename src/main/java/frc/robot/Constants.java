package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    /**
     * This class contains constants that regard the robot's configuration, such as
     * the hardware in it.
     */
    public static final class ConfigConstants {
        enum GyroType {
            NavX,
            Pigeon2,
        }

        public static final GyroType kRobotGyro = GyroType.NavX;
    }

    public static final class DriveConstants {
        /**
         * Constants that effect how the robot drives - chassis size, offsets & CAN IDs
         */
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // Radians per Second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        /* Chassis Configuration */
        public static final double kTrackWidth = Units.inchesToMeters(26); /* Distance Between right & left wheels */
        public static final double kWheelBase = Units.inchesToMeters(26); /* Distance between front & back wheels */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        /* Angular Offsets of Modules relative to Chassis in radians */
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kRearLeftChassisAngularOffset = Math.PI;
        public static final double kRearRightChassisAngularOffset = Math.PI / 2;

        /* SparkMAX CAN IDs */ // TODO: CHANGE TO REAL CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 31;
        public static final int kFrontRightDrivingCanId = 21;
        public static final int kRearRightDrivingCanId = 41;

        public static final int kFrontLeftTurningCanId = 12;
        public static final int kRearLeftTurningCanId = 32;
        public static final int kFrontRightTurningCanId = 22;
        public static final int kRearRightTurningCanId = 42;

        public static final boolean kGyroReversed = false;
    }

    /**
     * Constants for the swerve module - stuff that's the same for all 4
     */
    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        /* We use the High Speed pinions, therefore there are 14T */
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        /* Math to figure out speed and position of wheel */
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
        public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 1.03185; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxModuleSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 3 * Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1; /* Controls the Rotation of the Robot during Auton */

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
        public static final int NEO_CURRENT_LIMIT = 60;
    }

    public static final class ShooterConstants {
        public static final int kShootMotorID = 2;
        public static final int kFeedMotorID = 3;
        public static final int kRollerClawMotorID = 4;
        public static final double kFeedSpeed = 1;
        public static final double kShootSpeed = 1;
        public static final double kRollerClawSpeed = 0.5;
    }

    public static final class ClimberConstants {
        public static final int kClimbMotorID = 50; // TODO set to the real climber motor id
        public static final double kpPos = 10.0;
        public static final double kiPos = 0.0;
        public static final double kdPos = 1.0;
        public static final double feedForward = 0.617753;
        public static final double gravityCompensation = 0.059;
        public static final double max_vel = 0.25;
        public static final double max_accel = 2.50;
        public static final double simMeasurementStdDev = 0.0;
        public static final int CLIMBER_MOTOR_ID = 7;
        public static final double MIN_CLIMBER_HEIGHT = 0.0;
        public static final double MAX_CLIMBER_HEIGHT = 0.75;
        public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
        public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(0.5);
        public static final double VELOCITY_TOLERANCE = max_vel / 50.0;
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        /*
         * Camera Transforms - These are mapped from center of robot to the middle of
         * the lens on the camera.
         */
        public static final Transform3d kFrontCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(0.0), Units.inchesToMeters(20.75)),
                new Rotation3d(0.0, 0.0, 0.0));
        public static final Transform3d kLeftCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(-2.75), Units.inchesToMeters(12.5), Units.inchesToMeters(20.75)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(90)));
        public static final Transform3d kRightCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(-2.75), Units.inchesToMeters(-12.5),
                        Units.inchesToMeters(20.75)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90)));
        public static final Matrix<N3, N1> kLeftCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kLeftCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> kFrontCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kFrontCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> kRightCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kRightCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final double kRotateP = 0.005;
        public static final double kXTranslateP = 0.5;
        public static final double kYTranslateP = 0.25;

    }

    public static final class ScoringConstants {
        public static final Pose2d kTestingPose = new Pose2d(5, 5, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d kLeftSource = new Pose2d(15.88, 1.44, new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d kSpeakerCenter = new Pose2d(1.67, 5.3, new Rotation2d(Units.degreesToRadians(180)));
    }

    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
}
