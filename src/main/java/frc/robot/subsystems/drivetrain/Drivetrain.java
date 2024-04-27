// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;
import frc.utils.SwerveUtils;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class Drivetrain extends SubsystemBase {
    private Field2d m_field;
    private PathConstraints constraints;
    private Alliance ally;

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_rearLeft;
    private SwerveModule m_rearRight;

    private SwerveDrivePoseEstimator m_poseEstimator;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    private double m_currentRotationRate = 0.0;
    private double desiredAngle = 0;

    private Rotation2d lastAngle = new Rotation2d();

    private ChassisSpeeds relativeRobotSpeeds;

    /** Creates a new Drivetrain. */
    public Drivetrain(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_rearLeft,
            SwerveModule m_rearRight) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_rearLeft = m_rearLeft;
        this.m_rearRight = m_rearRight;

        m_field = new Field2d();
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(Gyro.getYaw()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                }, new Pose2d());
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0, 0.1), // Translation
                        new PIDConstants(0.975, 0, 0), // Rotation
                        DriveConstants.kMaxModuleSpeed,
                        Units.inchesToMeters(18.42), /* Distance from furthest module to robot center in meters */
                        new ReplanningConfig()),

                () -> {
                    // Basically flips the path for path planner depending on alliance(Origin is
                    // Blue Alliance)
                    var alliance = DriverStation.getAlliance();

                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },

                this);
        constraints = new PathConstraints(2.5, 5, 540, 720);

        publisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Drivetrain/SwerveStates", SwerveModuleState.struct)
                .publish();
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Robot/Field", m_field);
        m_field.setRobotPose(getPose());
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
        publisher.set(swerveModuleStates);
        m_poseEstimator.updateWithTime(MathSharedStore.getTimestamp(), Rotation2d.fromDegrees(Gyro.getYaw()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        updateVisionEstPose();
        if (Robot.isSimulation()) {
            double angleChange = DriveConstants.kDriveKinematics
                    .toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond
                    * (0.02);
            lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
            Gyro.setYaw(lastAngle.getDegrees());
        }
        updateModules();

    }

    public void drive(double x, double y, double rot, boolean fieldOriented) {

        double newRotRate = 0;
        double xSpeedCommanded;
        double ySpeedCommanded;
        double currentAngle = (Gyro.getYaw());

        if (currentAngle == 0) {
            desiredAngle = 0;
        }

        if (rot == 0 && (x != 0 | y != 0)) {
            newRotRate = 0;
            if (Math.abs(desiredAngle - currentAngle) > 1) {
                newRotRate = (2.0 * (desiredAngle - currentAngle)) % 360 / 360;
            }
        } else {
            newRotRate = rot;
            desiredAngle = currentAngle;
        }

        xSpeedCommanded = x;
        ySpeedCommanded = y;
        m_currentRotationRate = newRotRate;

        double xSpeedDelivered = xSpeedCommanded * RobotConstants.kMaxSpeed;
        double ySpeedDelivered = ySpeedCommanded * RobotConstants.kMaxSpeed;
        double rotRateDelivered = m_currentRotationRate * RobotConstants.kMaxRotationSpeed;

        relativeRobotSpeeds = fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered,
                        Rotation2d.fromDegrees(Gyro.getYaw()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);

        relativeRobotSpeeds = ChassisSpeeds.discretize(relativeRobotSpeeds, 0.02);

        SmartDashboard.putNumber("Drivetrain/X Velocity", relativeRobotSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Y Velocity", relativeRobotSpeeds.vyMetersPerSecond);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(relativeRobotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setZero() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, RobotConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
                m_rearLeft.getState(), m_rearRight.getState());
    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
                Rotation2d.fromDegrees(Gyro.getYaw()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
        System.out.println("Encoders Reset");
    }

    public Command pathFindtoPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public void setAlliance(Alliance ally) {
        this.ally = ally;
    }

    public boolean getWingStatus() {
        if (ally == Alliance.Red) {
            if (getPose().getX() >= FieldConstants.kRedWingBorder) {
                return true;
            } else {
                return false;
            }
        } else {
            if (getPose().getX() >= FieldConstants.kBlueWingBorder) {
                return true;
            } else {
                return false;
            }
        }
    }

    public void updateModules() {
        m_frontLeft.updateInputs();
        m_frontRight.updateInputs();
        m_rearLeft.updateInputs();
        m_rearRight.updateInputs();
    }

    public void updateVisionEstPose() {
        if (Vision.getFrontCamConnected()) {
            var frontCamEst = Vision.getFrontCamPose();
            frontCamEst.ifPresent(
                    est -> {
                        var frontCamEstPose = est.estimatedPose.toPose2d();
                        var frontCamEstStdDevs = Vision.getFrontEstStdDevs(frontCamEstPose);
                        m_poseEstimator.addVisionMeasurement(frontCamEstPose, est.timestampSeconds, frontCamEstStdDevs);
                    });
        }

        if (Vision.getLeftCamConnected()) {
            var leftCamEst = Vision.getLeftCamPose();
            leftCamEst.ifPresent(
                    est -> {
                        var leftCamEstPose = est.estimatedPose.toPose2d();
                        var leftCamEstStdDevs = Vision.getLeftEstStdDevs(leftCamEstPose);
                        m_poseEstimator.addVisionMeasurement(leftCamEstPose, est.timestampSeconds, leftCamEstStdDevs);
                    });
        }

        if (Vision.getRightCamConnected()) {
            var rightCamEst = Vision.getRightCamPose();
            rightCamEst.ifPresent(
                    est -> {
                        var rightCamEstPose = est.estimatedPose.toPose2d();
                        var rightCamEstStdDevs = Vision.getRightEstStdDevs(rightCamEstPose);
                        m_poseEstimator.addVisionMeasurement(rightCamEstPose, est.timestampSeconds, rightCamEstStdDevs);
                    });
        }
    }
}
