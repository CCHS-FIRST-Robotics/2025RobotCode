package frc.robot.subsystems.poseEstimator;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;

public class CameraIOPhotonVision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final String cameraName;
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    private final String cameraPrefix;
    private Matrix<N3, N1> currentEstimationStdDevs = VirtualConstants.SingleTagStdDevs; 

    public CameraIOPhotonVision(
            PhotonCamera camera,
            String cameraName,
            Transform3d cameraTransform) {
        this.camera = camera;
        this.cameraName = cameraName;
        this.poseEstimator = new PhotonPoseEstimator(
                PhysicalConstants.TagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d() 
        );
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        this.cameraPrefix = "Vision/" + cameraName + "/";
        System.out.println(cameraName + " Camera Initialized");
    }

    public void periodic() {
        Logger.recordOutput(cameraPrefix + "connected", camera.isConnected());

        PhotonPipelineResult latestResult = camera.getLatestResult();
        latestEstimatedPose = poseEstimator.update(latestResult);
        updateEstimationStdDevs(latestEstimatedPose, latestResult.getTargets());

        if (latestEstimatedPose.isPresent()) {
            Logger.recordOutput(cameraPrefix + "VisionPose2d", latestEstimatedPose.get().estimatedPose.toPose2d());
            Logger.recordOutput(cameraPrefix + "VisionPose3d", latestEstimatedPose.get().estimatedPose);
        } else {
            Logger.recordOutput(cameraPrefix + "VisionPose2d", new Pose2d());
            Logger.recordOutput(cameraPrefix + "VisionPose3d", new Pose3d());
        }

        Logger.recordOutput(cameraPrefix + "NumTags", latestResult.getTargets().size());
        Logger.recordOutput(cameraPrefix + "Timestamp", latestResult.getTimestampSeconds());
        System.out.println("StdDEV" + getEstimationStdDevs());
        Logger.recordOutput(cameraPrefix + "StdDevx", getEstimationStdDevs().get(0, 0));
        Logger.recordOutput(cameraPrefix + "StdDevy", getEstimationStdDevs().get(1, 0));
        Logger.recordOutput(cameraPrefix + "StdDevz", getEstimationStdDevs().get(2, 0));
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        if (latestEstimatedPose.isPresent()){
            return latestEstimatedPose.get();
        }
        return null;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            currentEstimationStdDevs = VirtualConstants.SingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VirtualConstants.SingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                currentEstimationStdDevs = VirtualConstants.SingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VirtualConstants.MultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 6)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                currentEstimationStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentEstimationStdDevs;
    }
}