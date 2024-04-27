package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
    public PhotonPipelineResult getFrontCameraResult();

    public PhotonPipelineResult getLeftCameraResult();

    public PhotonPipelineResult getRightCameraResult();

    public Optional<EstimatedRobotPose> getFrontEstPose();

    public Optional<EstimatedRobotPose> getLeftEstPose();

    public Optional<EstimatedRobotPose> getRightEstPose();

    public Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d estimatedPose);

    public Matrix<N3, N1> getLeftEstimationStdDevs(Pose2d estimatedPose);

    public Matrix<N3, N1> getRightEstimationStdDevs(Pose2d estimatedPose);

    public boolean[] checkCams();
}
