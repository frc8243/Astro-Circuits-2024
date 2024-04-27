package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants.MotorConstants;

public class DrivetrainConstants {
        public static final class DriveConstants {
                public static final double kDirectionSlewRate = 1.5; // radians per second
                public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.5; // percent per second (1 = 100%)

                /* Chassis Configuration */
                public static final double kTrackWidth = Units.inchesToMeters(26); /*
                                                                                    * Distance Between right & left
                                                                                    * wheels
                                                                                    */
                public static final double kWheelBase = Units.inchesToMeters(26); /*
                                                                                   * Distance between front & back
                                                                                   * wheels
                                                                                   */
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

                /* SparkMAX CAN IDs */
                public static final int kFrontLeftDrivingCanId = 11;
                public static final int kRearLeftDrivingCanId = 31;
                public static final int kFrontRightDrivingCanId = 21;
                public static final int kRearRightDrivingCanId = 41;

                public static final int kFrontLeftTurningCanId = 12;
                public static final int kRearLeftTurningCanId = 32;
                public static final int kFrontRightTurningCanId = 22;
                public static final int kRearRightTurningCanId = 42;

                public static final boolean kGyroReversed = false;

                /**
                 * This is a tricky one, this isn't the max *driving* speed, this is the max
                 * *total* velocity of the module.
                 * Pathplanner uses this, it should be much more than the max speed there.
                 * The number is in meters per second (m/s)
                 */
                public static final double kMaxModuleSpeed = 4.5;
        }

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
                public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNEOFreeSpeed / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kTurningMotorReduction = 9424d / 203;
                /* Math to figure out speed and position of wheel */
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
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

                public static final int kDrivingMotorCurrentLimit = 60; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }
}
