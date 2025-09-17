package frc.robot.subsystems.poseEstimator.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.PoseDataEntry;
import frc.robot.constants.*;

public class Vision {
    private final SwerveDrivePoseEstimator visionEstimator;
    private double latestTimestamp = 0;

    private final CameraIO[] cameraIOs;
    private final CameraIOInputsAutoLogged[] cameraIOInputs = new CameraIOInputsAutoLogged[VirtualConstants.NUM_CAMERAS];

    public Vision(
        CameraIO[] ios
    ) {
        visionEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            new SwerveModulePosition[4],
            new Pose2d(),
            VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE),
            VirtualConstants.SINGLE_TAG_STD_DEVS // doesn't really matter since it's always overridden below
        );

        this.cameraIOs = ios;

        for(int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            cameraIOInputs[i] = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(){
        for (int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            cameraIOs[i].updateInputs(cameraIOInputs[i]);
            for (PoseDataEntry pose : cameraIOInputs[i].visionPoseData) {
                visionEstimator.updateWithTime( // this doesn't update anything since stateStdDevs is Integer.MAX_VALUE
                    Timer.getFPGATimestamp(),
                    new Rotation2d(), 
                    new SwerveModulePosition[4] // ! changed; test if still works as intended
                );
                visionEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
                latestTimestamp = Math.max(pose.getTimestamp(), latestTimestamp);
            }
            Logger.processInputs("cameras/" + VirtualConstants.CAMERA_LOGGER_NAMES[i], cameraIOInputs[i]);
        }
    }

    public Pose2d getVisionEstimate(){
        return visionEstimator.getEstimatedPosition();
    }

    public double getLatestTimeStamp() {
        return latestTimestamp;
    }
}