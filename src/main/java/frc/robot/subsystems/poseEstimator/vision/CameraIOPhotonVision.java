package frc.robot.subsystems.poseEstimator.vision;

import static edu.wpi.first.units.Units.*;

import java.util.*;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.*;

public class CameraIOPhotonVision implements CameraIO{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    public CameraIOPhotonVision(int index) {
        this.camera = new PhotonCamera(PhysicalConstants.cameraNames[index]);
        this.poseEstimator = new PhotonPoseEstimator(
            PhysicalConstants.APRILTAG_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            PhysicalConstants.cameraTransforms[index]
        );

        // ! see if periodic only runs during enabled, and then just change it there instead of drilling a big hole through the code from robot.java
        // https://discord.com/channels/176186766946992128/528555967827148801/1367696455963381793
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // ! switch to PNP_DISTANCE_TRIG_SOLVE after enabled

        // ! only needs to be called once
        // https://github.com/PhotonVision/photonvision/pull/1662
        // NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) { // ! william used to have code that would show the estimate of each camera, you should put it back
        inputs.connected = camera.isConnected();
        
        // update pose data array
        ArrayList<PoseDataEntry> visionPoseData = new ArrayList<PoseDataEntry>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEstimate = poseEstimator.update(result);
            if(currentEstimate.isPresent()){
                updateStdDevs(currentEstimate.get(), result.getTargets());
                latestEstimatedPose = currentEstimate; // Update the latest estimate
                visionPoseData.add(new PoseDataEntry(currentEstimate.get().estimatedPose, result.getTimestampSeconds(), stdDevs));
            }
        }
        inputs.visionPoseData = visionPoseData;
    }

    private void updateStdDevs(
        EstimatedRobotPose estimatedPose, 
        List<PhotonTrackedTarget> targets
    ) {
        var estStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        double avgAmbugity = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var PhotonTarget : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(PhotonTarget.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            avgAmbugity += PhotonTarget.getPoseAmbiguity();
        }

        // ! WALK THROUGH WILLIAM'S LOGIC WITH HIM
        if (numTags == 0) {
            // No tags visible use single-tag std devs
            stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
            return;
        } 
        
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        avgAmbugity /= numTags;
        // MultiTag std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = PhysicalConstants.MULTI_TAG_STD_DEVS;
        }
        // Increase std devs based on (average) distance
        if ((numTags == 1 && avgDist > PhysicalConstants.TOO_FAR_AWAY ) 
            || !inField(estimatedPose) 
            || noisyPose(estimatedPose)
        ) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / PhysicalConstants.DISTANCE_WEIGHT));
            if (avgAmbugity > .2) {
                estStdDevs = estStdDevs.plus(avgAmbugity);
            }
        }

        stdDevs = estStdDevs;

    }

    private boolean inField(EstimatedRobotPose robotpose){
        Pose2d pose = robotpose.estimatedPose.toPose2d();
        return pose.getX() > 0 
            && pose.getY() > 0 
            && pose.getX() < PhysicalConstants.APRILTAG_LAYOUT.getFieldLength() 
            && pose.getY() < PhysicalConstants.APRILTAG_LAYOUT.getFieldWidth();
    }

    private boolean noisyPose(EstimatedRobotPose robotpose){
        EstimatedRobotPose lastRobotPose = latestEstimatedPose.get();
        Pose2d currentPose = robotpose.estimatedPose.toPose2d();
        Pose2d lastPose = lastRobotPose.estimatedPose.toPose2d();
        if (lastPose == null){
            return false;
        }
        Transform2d movement = currentPose.minus(lastPose);
        double time = robotpose.timestampSeconds - lastRobotPose.timestampSeconds;
        double maxXY = PhysicalConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond) * time;
        if(movement.getX() > maxXY || movement.getY() > maxXY){
            return true;
        }
        return false;
    }
}