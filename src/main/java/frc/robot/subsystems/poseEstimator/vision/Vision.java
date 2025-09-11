package frc.robot.subsystems.poseEstimator.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.PoseDataEntry;
import frc.robot.constants.*;

// go to 10.32.05.16:5800 for front opi5 dashboard
// go to 10.32.05.17:5800 for back opi5 dashboard

public class Vision {
    private final SwerveDrivePoseEstimator visionEstimator;
    private final CameraIO[] cameraIOs;
    private final CameraIOInputsAutoLogged[] cameraIOInputs = new CameraIOInputsAutoLogged[VirtualConstants.NUM_CAMERAS];

    private final Drive drive;

    public Vision(
        CameraIO[] ios,
        Drive drive
    ) {
        visionEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d(),
            VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE),
            VirtualConstants.SINGLE_TAG_STD_DEVS
        );

        this.cameraIOs = ios;

        for(int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            cameraIOInputs[i] = new CameraIOInputsAutoLogged();
        }

        this.drive = drive;
    }

    public void periodic(){
        for (int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            cameraIOs[i].updateInputs(cameraIOInputs[i]);
            for (PoseDataEntry pose : cameraIOInputs[i].visionPoseData) {
                visionEstimator.updateWithTime(
                    Timer.getFPGATimestamp(),
                    new Rotation2d(),
                    drive.getModulePositions()
                );
                visionEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), VecBuilder.fill(0, 0, 0));
            }
            Logger.processInputs("cameras/" + VirtualConstants.CAMERA_LOGGER_NAMES[i], cameraIOInputs[i]);
        }
    }

    public Pose2d getEstimatedPosition(){
        return visionEstimator.getEstimatedPosition();
    }
}