package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionReal implements VisionIO {
    private PhotonCamera frontCam;
    private PhotonCamera leftCam;
    private PhotonCamera rightCam;
    private PhotonPoseEstimator frontCamEstimator;
    private PhotonPoseEstimator leftCamEstimator;
    private PhotonPoseEstimator rightCamEstimator;
    private boolean[] camStatuses = new boolean[2];

    public VisionReal() {
        frontCam = new PhotonCamera("frontCamera");
        leftCam = new PhotonCamera("leftCamera");
        rightCam = new PhotonCamera("rightCamera");

        frontCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, VisionConstants.kFrontCamtoRobot);
        leftCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, VisionConstants.kLeftCamtoRobot);
        rightCamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, VisionConstants.kRightCamtoRobot);

    }

    @Override
    public boolean[] checkCams() {
        camStatuses[0] = frontCam.isConnected();
        camStatuses[1] = leftCam.isConnected();
        camStatuses[2] = rightCam.isConnected();
        return camStatuses;
    }

    @Override
    public PhotonPipelineResult getFrontCameraResult() {
        return frontCam.getLatestResult();
    }

    @Override
    public PhotonPipelineResult getLeftCameraResult() {
        return leftCam.getLatestResult();
    }

    @Override
    public PhotonPipelineResult getRightCameraResult() {
        return rightCam.getLatestResult();
    }

    @Override
    public Optional<EstimatedRobotPose> getFrontEstPose() {
        var estPose = frontCamEstimator.update();
        return estPose;
    }

    @Override
    public Optional<EstimatedRobotPose> getLeftEstPose() {
        var estPose = leftCamEstimator.update();
        return estPose;
    }

    @Override
    public Optional<EstimatedRobotPose> getRightEstPose() {
        var estPose = rightCamEstimator.update();
        return estPose;
    }

    @Override
    public Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kFrontCamSingleStdDevs;
        var targets = frontCam.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = leftCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VisionConstants.kFrontCamMultiStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    @Override
    public Matrix<N3, N1> getLeftEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kLeftCamSingleStdDevs;
        var targets = leftCam.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = leftCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VisionConstants.kLeftCamMultiStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    @Override
    public Matrix<N3, N1> getRightEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kRightCamSingleStdDevs;
        var targets = rightCam.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = rightCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VisionConstants.kRightCamMultiStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
