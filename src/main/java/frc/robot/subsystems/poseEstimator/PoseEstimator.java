package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private Pose2d fieldPosition = new Pose2d();
    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private Transform2d[] tagOffsets = new Transform2d[0];

    private final Drive drive;

    public PoseEstimator(
        GyroIO gyroIO, 
        CameraIO cameraIO, 
        Drive drive
    ) {
        this.gyroIO = gyroIO;
        this.cameraIO = cameraIO;

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
    }

    @Override
    public void periodic() {
        // inputs
        gyroIO.updateInputs(gyroInputs);
        cameraIO.updateInputs(cameraInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);
        Logger.processInputs("poseEstimator/camera", cameraInputs);
        
        // odometry
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        // vision
        visionEstimate = cameraInputs.poseEstimate;
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    
        tagOffsets = cameraInputs.tagOffsets;
    }

    public void resetPosition(Pose2d pose) {
        fieldPosition = pose;
        odometryEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
        combinedEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
    }

    // @SuppressWarnings("unused")
    private Pose2d getOdometryPose() {
        return odometryEstimator.getEstimatedPosition();
    }

    @SuppressWarnings("unused")
    private Pose2d getVisionPose() {
        return visionEstimate;
    }

    @SuppressWarnings("unused")
    private Pose2d getCombinedPose() {
        return combinedEstimator.getEstimatedPosition();
    }

    public Pose2d getPose() { // change function here for different poseEstimate
        return getOdometryPose();
    }

    public Rotation2d getRawYaw() {
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }

    public Transform2d getSpecificTagOffset(int id) {
        return tagOffsets[id - 1];
    }
}