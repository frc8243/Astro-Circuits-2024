package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionSim implements VisionIO {

    @Override
    public PhotonPipelineResult getFrontCameraResult() {
        return null;

    }

    @Override
    public PhotonPipelineResult getLeftCameraResult() {
        return null;

    }

    @Override
    public PhotonPipelineResult getRightCameraResult() {
        return null;

    }

    @Override
    public Optional<EstimatedRobotPose> getFrontEstPose() {
        return null;

    }

    @Override
    public Optional<EstimatedRobotPose> getLeftEstPose() {
        return null;

    }

    @Override
    public Optional<EstimatedRobotPose> getRightEstPose() {
        return null;

    }

    @Override
    public Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d estimatedPose) {
        return null;

    }

    @Override
    public Matrix<N3, N1> getLeftEstimationStdDevs(Pose2d estimatedPose) {
        return null;

    }

    @Override
    public Matrix<N3, N1> getRightEstimationStdDevs(Pose2d estimatedPose) {
        return null;

    }

    @Override
    public boolean[] checkCams() {
        return new boolean[3];

    }

}
