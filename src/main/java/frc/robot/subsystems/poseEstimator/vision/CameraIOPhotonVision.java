package frc.robot.subsystems.poseEstimator.vision;

import static edu.wpi.first.units.Units.*;

import java.util.*;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.*;

// ! naming

public class CameraIOPhotonVision implements CameraIO{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
    private EstimatedRobotPose latestEstimatedPose;

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
        
        // update pose data
        ArrayList<PoseDataEntry> visionPoseData = new ArrayList<PoseDataEntry>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEstimate = poseEstimator.update(result);
            if(currentEstimate.isPresent()){
                updateStdDevs(currentEstimate.get(), result.getTargets());
                latestEstimatedPose = currentEstimate.get(); // Update the latest estimate
                visionPoseData.add(new PoseDataEntry(currentEstimate.get().estimatedPose, result.getTimestampSeconds(), stdDevs));
            }
        }
        inputs.visionPoseData = visionPoseData;
    }

    private void updateStdDevs(
        EstimatedRobotPose currentEstimate, 
        List<PhotonTrackedTarget> targets
    ) {
        Matrix<N3, N1> estStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
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
        averageDistance /= numTags;
        averageAmbiguity /= numTags;

        switch(numTags){
            case 0:
                stdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
                break;
            case 1: 

            default: // in this case, if numTags > 1
                estStdDevs = PhysicalConstants.MULTI_TAG_STD_DEVS;

        }

        // ! go through this with william !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        // Increase std devs based on (average) distance
        if ((numTags == 1 && averageDistance > PhysicalConstants.TOO_FAR_AWAY ) 
            || !inField(currentEstimate) 
            || noisyPose(currentEstimate)
        ) {
            estStdDevs = VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
        }
        else {
            // ! magic equation go brr
            estStdDevs = estStdDevs.times(1 + (averageDistance * averageDistance / PhysicalConstants.DISTANCE_WEIGHT));
            if (averageAmbiguity > .2) {
                estStdDevs = estStdDevs.plus(averageAmbiguity);
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
        EstimatedRobotPose lastRobotPose = latestEstimatedPose;
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