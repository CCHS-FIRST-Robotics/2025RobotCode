package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIO cameraIO;
    private final HashMap<Integer, double[]> tagMap = new HashMap<Integer, double[]>();
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;
    private Pose2d fieldPosition = new Pose2d();

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
        // gyro
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);

        // camera
        cameraIO.updateInputs(cameraInputs);
        Logger.processInputs("poseEstimator/camera", cameraInputs);
        
        // pose
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        visionEstimate = updateVisionEstimate();
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp()); 
        
        Logger.recordOutput("outputs/poseEstimator/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/poseEstimator/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public Pose2d updateVisionEstimate() { // ! 
        int tagCount = 0;
        double accumulatedX = 0;
        double accumulatedY = 0;
        for (int i = 1; i < cameraInputs.tagArray.length; i += 4) { // tags[0] is the packetId
            int id = (int) cameraInputs.tagArray[i];
            double distance = cameraInputs.tagArray[i+3]; // meters
            double tagAngle = cameraInputs.tagArray[i+1]; // radians
            double robotAngle = getRawYaw().getRadians();

            double angleToTag = tagAngle - robotAngle;
            double xDistance = Math.sin(angleToTag) * distance;
            double yDistance = Math.cos(angleToTag) * distance;

            tagMap.put(id, new double[] {xDistance, yDistance, angleToTag}); // ! adjust for jetson offset

            Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(id);
            accumulatedX += taglocation.getX() - xDistance;
            accumulatedY += taglocation.getY() - yDistance;

            tagCount++;
        }

        if (tagCount == 0) {
            return new Pose2d();    
        }

        return new Pose2d(accumulatedX / tagCount, accumulatedY / tagCount, getRawYaw()).plus(PhysicalConstants.JETSON_OFFSET);
    }

    public void resetPosition(Pose2d pose) {
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

    public Pose2d getPose() {
        return getOdometryPose();
    }

    public Rotation2d getRawYaw() {
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }

    public double[] getSpecificTag(int id) {
        return tagMap.get(id);
    }
}