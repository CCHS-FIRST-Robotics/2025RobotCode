package frc.robot.subsystems.poseEstimator.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.vision.CameraIOPhotonVision.PoseDataEntry;
import frc.robot.constants.PhysicalConstants;

public class Vision {
    private final SwerveDrivePoseEstimator visionEstimator;
    private final CameraIOPhotonVision[] ios = new CameraIOPhotonVision[4]; // ! decide on names someday

    public Vision(Drive drive) {
        visionEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d()
        );

        for (int i = 0; i < PhysicalConstants.numCameras; i++) {
            ios[i] = new CameraIOPhotonVision(PhysicalConstants.cameraNames[i], PhysicalConstants.cameraTransforms[i], drive);
        }
    }

    public void periodic(){
        for (CameraIOPhotonVision io : ios) {
            io.periodic();
            for (PoseDataEntry pose : io.getVisionPoseData()){
                visionEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }
    }
    
    public void SetFallbackStrategy(PoseStrategy posestrat) {
        for (CameraIOPhotonVision io : ios) {
            io.FallbackStrategy(posestrat);
        }
    }

    public Pose2d getEstimatedPosition(){
        return visionEstimator.getEstimatedPosition();
    }
}