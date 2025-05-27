package frc.robot.subsystems.poseEstimator.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.PoseDataEntry;
import frc.robot.constants.PhysicalConstants;

public class Vision {
    private final SwerveDrivePoseEstimator visionEstimator;
    private final CameraIO[] cameraIOs;
    private final CameraIOInputsAutoLogged[] cameraIOInputs = new CameraIOInputsAutoLogged[PhysicalConstants.NUM_CAMERAS];

    public Vision(
        CameraIO[] ios,
        Drive drive
    ) {
        visionEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d()
        );

        this.cameraIOs = ios;
    }

    public void periodic(){
        for (int i = 0; i < PhysicalConstants.NUM_CAMERAS; i++) {
            cameraIOs[i].updateInputs(cameraIOInputs[i]);
            for (PoseDataEntry pose : cameraIOInputs[i].visionPoseData) {
                visionEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }
    }

    public Pose2d getEstimatedPosition(){
        return visionEstimator.getEstimatedPosition();
    }
}