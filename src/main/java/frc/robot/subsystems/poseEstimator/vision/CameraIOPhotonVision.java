package frc.robot.subsystems.poseEstimator.vision;

import java.util.*;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.*;

public class CameraIOPhotonVision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    private final String cameraPrefix;
    private final SwerveDrivePoseEstimator cameraEstimator;
    private Matrix<N3, N1> currentEstimationStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
    private final ArrayList<PoseDataEntry> visionPoseData = new ArrayList<>();

    private final Drive drive;

    public CameraIOPhotonVision (
        String cameraName,
        Transform3d cameraTransform, 
        Drive drive
    ) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraPrefix = "outputs/poseEstimator/cameras/" + cameraName + "/";
        this.poseEstimator = new PhotonPoseEstimator(
            PhysicalConstants.APRILTAG_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            cameraTransform
        );

        // ! see if periodic only runs during enabled, and then just change it there instead of drilling a big hole through the code from robot.java

        // https://discord.com/channels/176186766946992128/528555967827148801/1367696455963381793
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // ! switch to PNP_DISTANCE_TRIG_SOLVE after enabled

        this.cameraEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d(), 
            VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE),
            PhysicalConstants.SINGLE_TAG_STD_DEVS
        );

        this.drive = drive;

        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true); // https://github.com/PhotonVision/photonvision/pull/1662
    }
    
    public void periodic() { // ! shoud this all be like an updateInputs thing
        cameraEstimator.update(new Rotation2d(), drive.getModulePositions());
        updateVision();
        
        Logger.recordOutput(cameraPrefix + "connected", camera.isConnected());

        for (PoseDataEntry pose : visionPoseData){
            cameraEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
        }
        if (latestEstimatedPose.isPresent()) {
            List<PhotonTrackedTarget> targets = latestEstimatedPose.get().targetsUsed;
        
            for(int j=0; j<= targets.size() -1; j++){
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/ID", targets.get(j).fiducialId);
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/Transform3d", targets.get(j).getBestCameraToTarget());
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/Ambugity", targets.get(j).getPoseAmbiguity());
            }
            Logger.recordOutput(cameraPrefix + "VisionPose2d", cameraEstimator.getEstimatedPosition());
        }
    }

    private void updateVision() {
        visionPoseData.clear();
        for (var result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEst = poseEstimator.update(result);
            updateEstimationStdDevs(currentEst, result.getTargets());
            if (currentEst.isPresent()) {
                latestEstimatedPose = currentEst; // Update the latest estimate
                visionPoseData.add(new PoseDataEntry(currentEst.get().estimatedPose, result.getTimestampSeconds(), getEstimationStdDevs()));
            }
        }
    }

    public void FallbackStrategy(PoseStrategy posestrat) {
        poseEstimator.setMultiTagFallbackStrategy(posestrat);
    }

    public ArrayList<PoseDataEntry> getVisionPoseData() {
        return visionPoseData;
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
            currentEstimationStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
            return;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var PhotonTarget : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(PhotonTarget.getFiducialId());
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
                // No tags visible use single-tag std devs
                currentEstimationStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // MultiTag std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = PhysicalConstants.SINGLE_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > PhysicalConstants.TOO_FAR_AWAY || !InField(estimatedPose.get()) )
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / PhysicalConstants.DISTANCE_WEIGHT));
                currentEstimationStdDevs = estStdDevs;
            }
        }
    }

    
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentEstimationStdDevs;
    }

    private boolean InField(EstimatedRobotPose robotpose){
        Pose2d pose = robotpose.estimatedPose.toPose2d();
        return pose.getX() > 0 && pose.getY() > 0 && pose.getX() < PhysicalConstants.APRILTAG_LAYOUT.getFieldLength() && pose.getY() < PhysicalConstants.APRILTAG_LAYOUT.getFieldWidth();
    }

    class PoseDataEntry {
        private final Pose3d robotPose;
        private final double timestamp;
        private final Matrix<N3, N1> standardDeviation;

        public PoseDataEntry(Pose3d robotPose, double timestamp, Matrix<N3, N1> standardDeviation) {
            this.robotPose = robotPose;
            this.timestamp = timestamp;
            this.standardDeviation = standardDeviation;
        }

        public Pose3d getRobotPose() {
            return robotPose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Matrix<N3, N1> getStandardDeviation() {
            return standardDeviation;
        }

        @Override
        public String toString() {
            return "[" + robotPose + ", timestamp=" + timestamp + ", stddev=" + standardDeviation + "]";
        }
    }
}