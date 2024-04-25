package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.*;

public class VisionConstants {
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        /*
         * Camera Transforms - These are mapped from center of robot to the middle of
         * the lens on the camera.
         */
        public static final Transform3d kFrontCamtoRobot = new Transform3d(
                        new Translation3d(Units.inchesToMeters(14.625), Units.inchesToMeters(0),
                                        Units.inchesToMeters(26)),
                        new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));
        public static final Transform3d kLeftCamtoRobot = new Transform3d(
                        new Translation3d(Units.inchesToMeters(1.875), Units.inchesToMeters(9.75),
                                        Units.inchesToMeters(8.75)),
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
