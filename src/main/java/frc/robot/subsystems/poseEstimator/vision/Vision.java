package frc.robot.subsystems.poseEstimator.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.PoseDataEntry;
import frc.robot.constants.*;

public class Vision {
    private final SwerveDrivePoseEstimator visionEstimator;
    private double latestTimestamp = 0;

    private final CameraIO[] ios;
    private final CameraIOInputsAutoLogged[] inputs = new CameraIOInputsAutoLogged[VirtualConstants.NUM_CAMERAS];

    public Vision(
        CameraIO[] ios
    ) {
        visionEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            PhysicalConstants.BLANK_MODULE_POSITIONS,
            new Pose2d(),
            VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE),
            VirtualConstants.SINGLE_TAG_STD_DEVS // not sure why this param exists because it's always overridden
        );

        this.ios = ios;

        for(int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            inputs[i] = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(){
        for (int i = 0; i < VirtualConstants.NUM_CAMERAS; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("cameras/" + VirtualConstants.CAMERA_LOGGER_NAMES[i], inputs[i]);
            
            // update visionEstimator based on new measurements
            for (PoseDataEntry pose : inputs[i].visionPoseData) {
                visionEstimator.updateWithTime( // this doesn't impact the pose estimate since stateStdDevs is Integer.MAX_VALUE
                    Timer.getFPGATimestamp(),
                    new Rotation2d(),
                    PhysicalConstants.BLANK_MODULE_POSITIONS
                );
                visionEstimator.addVisionMeasurement(pose.robotPose().toPose2d(), pose.timestamp(), pose.standardDeviation());
                latestTimestamp = Math.max(pose.timestamp(), latestTimestamp);
            }
        }
    }

    public Pose2d getVisionEstimate(){
        return visionEstimator.getEstimatedPosition();
    }

    public double getLatestTimeStamp() {
        return latestTimestamp;
    }
}