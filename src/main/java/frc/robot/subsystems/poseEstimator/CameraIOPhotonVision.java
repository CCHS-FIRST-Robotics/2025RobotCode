package frc.robot.subsystems.poseEstimator;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;



import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;

public class CameraIOPhotonVision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final String cameraName;
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    private final String cameraPrefix;
    private final SwerveDrivePoseEstimator VisionEstimator;
    private Matrix<N3, N1> currentEstimationStdDevs = VirtualConstants.SingleTagStdDevs;
    private final List<PoseDataEntry> VisionPoses = new ArrayList<>();

    public class PoseDataEntry {
        private final Pose3d robotPose;
        private final double timestamp;
        private final Matrix<N3, N1> standardDeviation;

        //! DIFFERENT CLASS
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
    //! Class ends 

    public CameraIOPhotonVision(
            PhotonCamera camera,
            String cameraName,
            Transform3d cameraTransform) 
        {
        this.camera = camera;
        this.cameraName = cameraName;
        this.poseEstimator = new PhotonPoseEstimator(
                PhysicalConstants.TagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransform
        );
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE); // play with this but start game with lowest ambugity for when disabled.
        this.VisionEstimator = new SwerveDrivePoseEstimator( //! sketchy but doesnt use swerve drive data(stddevs to high) so its just a visionePoseEstimator
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(999999, 999999, 999999),
            VirtualConstants.SingleTagStdDevs
        
        );
        // Hit the undocumented Photon Turbo Button™
        // https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
        this.cameraPrefix = "outputs/Vision/" + cameraName + "/";
        System.out.println(cameraName + " Camera Initialized");
    }
    
    public void periodic() {
        VisionEstimator.update(new Rotation2d(), getModulePositions());
        Logger.recordOutput(cameraPrefix + "connected", camera.isConnected());
        UpdateVision();
        

        for (PoseDataEntry pose : VisionPoses){
            VisionEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
        }
        if (latestEstimatedPose.isPresent()) {
            List<PhotonTrackedTarget> targets = latestEstimatedPose.get().targetsUsed;
        
            for(int j=0; j<= targets.size() -1; j++){
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/ID", targets.get(j).fiducialId);
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/Transform3d", targets.get(j).getBestCameraToTarget());
                Logger.recordOutput(cameraPrefix + " " + j + " tag seen/Ambugity", targets.get(j).getPoseAmbiguity());
            }
            Logger.recordOutput(cameraPrefix + "VisionPose2d", VisionEstimator.getEstimatedPosition());
        }
            
         //! didnt need these for troubleshooting but will leave for now   
        //     for(int i=0; i <= VisionPoses.size() - 1; i++){
        //         PoseDataEntry pose = VisionPoses.get(i);
        //         Logger.recordOutput(cameraPrefix + "VisionPose" + i + "Pose2d", pose.robotPose.toPose2d());
        //         Logger.recordOutput(cameraPrefix + "VisionPose" + i + "Std dev", pose.getStandardDeviation());
        //         Logger.recordOutput(cameraPrefix + "VisionPose" + i + "time", pose.getTimestamp());
                
        //     }
        // } else {
        //     Logger.recordOutput(cameraPrefix + "VisionPose2d", new Pose2d());
        //     Logger.recordOutput(cameraPrefix + "VisionPose3d", new Pose3d());
        // }

        // Logger.recordOutput(cameraPrefix + "StdDevx", getEstimationStdDevs().get(0, 0));
        // Logger.recordOutput(cameraPrefix + "StdDevy", getEstimationStdDevs().get(1, 0));
        // Logger.recordOutput(cameraPrefix + "StdDevz", getEstimationStdDevs().get(2, 0));
    }


   
    private void UpdateVision() {
        VisionPoses.clear();
        for (var result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> currentEst = poseEstimator.update(result);
            updateEstimationStdDevs(currentEst, result.getTargets());
            if (currentEst.isPresent()) {
                latestEstimatedPose = currentEst; // Update the latest estimate
                VisionPoses.add(new PoseDataEntry(currentEst.get().estimatedPose, result.getTimestampSeconds(), getEstimationStdDevs()));
            }
        }

    }


    public void FallbackStrategy(PoseStrategy posestrat) {
        poseEstimator.setMultiTagFallbackStrategy(posestrat);
    }


    public List<PoseDataEntry> getVisionPoses() {
        return VisionPoses;
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
            return;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VirtualConstants.SingleTagStdDevs;
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
                currentEstimationStdDevs = VirtualConstants.SingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // MultiTag std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VirtualConstants.MultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > VirtualConstants.ToFarAway || !InField(estimatedPose.get()) )
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / VirtualConstants.DistanceWeight));
                currentEstimationStdDevs = estStdDevs;
            }
        }
    }

    
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentEstimationStdDevs;
    }



    private boolean InField(EstimatedRobotPose robotpose){
        Pose2d pose = robotpose.estimatedPose.toPose2d();
        return pose.getX() > 0 && pose.getY() > 0 && pose.getX() < PhysicalConstants.TagLayout.getFieldLength() && pose.getY() < PhysicalConstants.TagLayout.getFieldWidth();
    }


    private SwerveModulePosition[] getModulePositions() {
       SwerveModulePosition moduleposit = new SwerveModulePosition();
       SwerveModulePosition[] ret = new SwerveModulePosition[4];
       for(int i = 0; i < 4; i++){
            ret[i] = moduleposit;
       }
       return ret;

       
    }







}