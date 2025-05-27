package frc.robot.subsystems.poseEstimator.vision;

import java.util.*;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import frc.robot.constants.*;

public class CameraIOPhotonVision implements CameraIO{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;

    public CameraIOPhotonVision(int index) {
        this.camera = new PhotonCamera(PhysicalConstants.cameraNames[index]);
        this.poseEstimator = new PhotonPoseEstimator(
            PhysicalConstants.APRILTAG_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            PhysicalConstants.cameraTransforms[index]
        );

        // ! see if periodic only runs during enabled, and then just change it there instead of drilling a big hole through the code from robot.java
        // https://discord.com/channels/176186766946992128/528555967827148801/1367696455963381793
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // ! switch to PNP_DISTANCE_TRIG_SOLVE after enabled

        // ! only needs to be called once, probably shouldn't be in this class
        // https://github.com/PhotonVision/photonvision/pull/1662
        // NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) { // ! william used to have code that would show the estimate of each camera, you should put it back
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

    private void updateStdDevs(
        EstimatedRobotPose currentEstimate, 
        List<PhotonTrackedTarget> targets
    ) {
        int numTags = 0;
        double averageDistance = 0;
        double averageAmbiguity = 0;
        for (PhotonTrackedTarget PhotonTarget : targets) {
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
                break;
            case 1: 
                stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS.times( // ! cough cough magic equation
                    1 + (averageDistance * averageDistance / PhysicalConstants.DISTANCE_WEIGHT)
                );
                break;
            default: // if numTags > 1
                averageDistance /= numTags;
                averageAmbiguity /= numTags;
                stdDevs = PhysicalConstants.MULTI_TAG_STD_DEVS;
                break;
        }

        if (averageAmbiguity > 0.2) { // "numbers above 0.2 are likely to be ambiguous"
            stdDevs = stdDevs.plus(averageAmbiguity);
        }
    }
}