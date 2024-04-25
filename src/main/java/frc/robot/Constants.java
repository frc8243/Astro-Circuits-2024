package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
     * the hardware in it
     */
    public static final class ConfigConstants {
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

    }

    public static final class OIConstants {
        public static final double kDriveDeadband = 0.1;
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
        public static final int kNeoCurrentLimit = 60;
        public static final int kNeo550CurrentLimit = 20;
    }

    public static final class ShooterConstants {
        public static final int kShootMotorID = 2;
        public static final int kFeedMotorID = 4;
        public static final int kRollerClawMotorID = 3;
        public static final double kFeedSpeed = 1;
        public static final double kShootSpeed = 1;
        
        public static final double kShooterReadySpeed = 5900;
        public static final double kP = 0.00025;
        public static final double kI = 0;
        public static final double kD = 0.002;
        public static final double kFF = 1 / NeoMotorConstants.kFreeSpeedRpm;
        public static final double kRPMTolerance = 150;
    }

    public static final class ClimberConstants {
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

    public static final class VisionConstants {
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        /*
         * Camera Transforms - These are mapped from center of robot to the middle of
         * the lens on the camera.
         */
        public static final Transform3d kFrontCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(14.625), Units.inchesToMeters(0), Units.inchesToMeters(26)),
                new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));
        public static final Transform3d kLeftCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(1.875), Units.inchesToMeters(9.75), Units.inchesToMeters(8.75)),
                new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(90)));
        public static final Transform3d kRightCamtoRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(1.9375), Units.inchesToMeters(-9.75),
                        Units.inchesToMeters(8.75)),
                new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-90)));
        public static final Matrix<N3, N1> kLeftCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kLeftCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> kFrontCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kFrontCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> kRightCamSingleStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kRightCamMultiStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final double kRotateP = 0.005;
        public static final double kXTranslateP = 0.5;
        public static final double kYTranslateP = 0.25;
        public static final int kBlueAmpTag = 6;
        public static final int kBlueLeftSourceTag = 2;
        public static final int kBlueRightSourceTag = 1;
        public static final int kBlueSpeakerTag = 7;
        public static final int kBlueSideSpeakerTag = 8;
        public static final int kRedSpeakerTag = 4;
        public static final int kRedSideSpeakerTag = 3;
        public static final int kRedLeftSourceTag = 10;
        public static final int kRedRightSourceTag = 9;
        public static final int kRedAmpTag = 5;

    }

    public static final class ScoringConstants {
        public static final Pose2d kTestingPose = new Pose2d(5, 5, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d kBlueLeftSource = new Pose2d(15.88, 1.44,
                new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d kBlueSpeakerCenter = new Pose2d(1.67, 5.3,
                new Rotation2d(Units.degreesToRadians(180)));
        public static final double kBlueSourceAngle = 120;
        public static final double kRedSourceAngle = -120;
        public static final double kSourceAlignmentTolerance = 2.5;
    }

}
