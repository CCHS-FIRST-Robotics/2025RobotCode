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
        System.out.println(camera.getName() + "Initilized");;

        // ! see if periodic only runs during enabled, and then just change it there instead of drilling a big hole through the code from robot.java
        // https://discord.com/channels/176186766946992128/528555967827148801/1367696455963381793
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // ! switch to PNP_DISTANCE_TRIG_SOLVE after enabled

        // ! only needs to be called once, probably shouldn't be in this class
        // https://github.com/PhotonVision/photonvision/pull/1662
        // NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) { 
        inputs.connected = camera.isConnected();

        
        ArrayList<PoseDataEntry> collectedVisionPoses = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEstimate = poseEstimator.update(result);
            if(currentEstimate.isPresent()){
                updateStdDevs(currentEstimate.get(), result.getTargets());
        
                collectedVisionPoses.add(new PoseDataEntry(currentEstimate.get().estimatedPose, result.getTimestampSeconds(), stdDevs));
            }
        }
        // Convert the ArrayList to a PoseDataEntry[] array and assign it to inputs.visionPoseData
        inputs.visionPoseData = collectedVisionPoses.toArray(new PoseDataEntry[0]);
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
        // Avoid division by zero if no tags are found
        if (numTags > 0) {
            averageDistance /= numTags;
            averageAmbiguity /= numTags;
        }


        switch(numTags) {
            case 0:
                // Use Double.MAX_VALUE for "no estimate" or very high uncertainty
                stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                break;
            case 1:
                stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS.times( // ! cough cough magic equation
                    1 + (averageDistance * averageDistance / PhysicalConstants.DISTANCE_WEIGHT)
                );
                if (averageAmbiguity > 0.2) { // "numbers above 0.2 are likely to be ambiguous"
                    stdDevs = stdDevs.plus(averageAmbiguity);
                }
                break;
            default: // if numTags > 1
                stdDevs = PhysicalConstants.MULTI_TAG_STD_DEVS;
                // You might still want to scale multi-tag stdDevs based on ambiguity or distance if desired
                // For example:
                // if (averageAmbiguity > 0.1) {
                //     stdDevs = stdDevs.plus(averageAmbiguity);
                // }
                break;
        }
    }
}
