package frc.robot.subsystems.poseEstimator.vision;

import java.util.*;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import frc.robot.constants.*;

// go to 10.32.05.16:5800 for front opi5 dashboard
// go to 10.32.05.17:5800 for back opi5 dashboard

public class CameraIOPhotonVision implements CameraIO{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> stdDevs = VirtualConstants.SINGLE_TAG_STD_DEVS;

    public CameraIOPhotonVision(int index) {
        this.camera = new PhotonCamera(VirtualConstants.CAMERA_PHOTONVISION_NAMES[index]);
        this.poseEstimator = new PhotonPoseEstimator(
            VirtualConstants.APRILTAG_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhysicalConstants.CAMERA_TRANSFORMS[index]
        );

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    private void updateStdDevs(
        EstimatedRobotPose currentEstimate, 
        List<PhotonTrackedTarget> targets
    ) {
        int numTags = 0;
        double averageDistance = 0;
        double averageAmbiguity = 0;
        for (PhotonTrackedTarget PhotonTarget : targets) { // get average distance and ambiguity
            numTags++;

            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(PhotonTarget.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            averageDistance += tagPose.get().toPose2d().getTranslation().getDistance(
                currentEstimate.estimatedPose.toPose2d().getTranslation()
            );

            averageAmbiguity += PhotonTarget.getPoseAmbiguity();
        }

        switch(numTags) {
            case 0:
                stdDevs = VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
                return;
            case 1: 
                stdDevs = VirtualConstants.SINGLE_TAG_STD_DEVS;
                break;
            default: // if numTags > 1
                stdDevs = VirtualConstants.MULTI_TAG_STD_DEVS;
                break;
        }

        averageDistance /= numTags;
        averageAmbiguity /= numTags;

        stdDevs = stdDevs.times( // ! magic equation
            1 + (averageDistance * averageDistance / VirtualConstants.DISTANCE_WEIGHT)
        );

        if (averageAmbiguity > 0.2) { // "numbers above 0.2 are likely to be ambiguous" - PhotonTarget.getPoseAmbiguity()
            stdDevs = stdDevs.plus(averageAmbiguity);
        }
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.connected = camera.isConnected();
        
        // update pose data
        ArrayList<PoseDataEntry> visionPoseData = new ArrayList<PoseDataEntry>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEstimate = poseEstimator.update(result);
            if(currentEstimate.isPresent()){
                updateStdDevs(currentEstimate.get(), result.getTargets());
                visionPoseData.add(new PoseDataEntry(currentEstimate.get().estimatedPose, result.getTimestampSeconds(), stdDevs));
            }
        }
        inputs.visionPoseData = visionPoseData.toArray(new PoseDataEntry[0]);
    }
}