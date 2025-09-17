package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final Odometry odometry;
    private final Vision vision;

    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private final Drive drive;

    public PoseEstimator(
        GyroIO gyroIO, 
        CameraIO[] cameraIOs, 
        Drive drive
    ) {
        odometry = new Odometry(gyroIO, drive);
        vision = new Vision(cameraIOs, drive);

        odometryEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d()
        );
        visionEstimate = new Pose2d();
        combinedEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d()
        );

        this.drive = drive;

        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true); // https://github.com/PhotonVision/photonvision/pull/1662
    }

    @Override
    public void periodic() {     
        odometry.periodic();
        vision.periodic();
        
        // odometry
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            odometry.getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        // vision
        visionEstimate = vision.getVisionEstimate(); // based on the last time it saw an apriltag
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            odometry.getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, vision.getLatestTimeStamp());
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public void resetPosition(Pose2d pose) {
        odometryEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
        visionEstimate = new Pose2d();
        combinedEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return getCombinedPose();
    }

    @SuppressWarnings("unused")
    private Pose2d getOdometryPose() {
        return odometryEstimator.getEstimatedPosition();
    }

    @SuppressWarnings("unused")
    private Pose2d getVisionPose() {
        return visionEstimate;
    }

    // @SuppressWarnings("unused")
    private Pose2d getCombinedPose() {
        return combinedEstimator.getEstimatedPosition();
    }
}