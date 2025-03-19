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
    private HashMap<Integer, double[]> tagOffsetMap = new HashMap<Integer, double[]>();
    private final int ticksToKeep = 100;  // ! figure out how big it should be
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
        
        for(int key : tagOffsetMap.keySet()) { // ! stays in logger even after removed from array
            Logger.recordOutput("outputs/poseEstimator/tagOffsetMap/" + Integer.toString(key), tagOffsetMap.get(key));
        }

        Logger.recordOutput("outputs/poseEstimator/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/poseEstimator/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public Pose2d updateVisionEstimate() {
        int tagCount = 0; // front, left, right, back, -25, -3, -3, 30
        double accumulatedX = 0;
        double accumulatedY = 0;
        HashMap<Integer, double[]> tempTagOffsetMap = new HashMap<Integer, double[]>();
        for (int i = 1; i < cameraInputs.tagArray.length; i += 4) { // tags[0] is the packetId
            // get networktables values
            int id = (int) cameraInputs.tagArray[i];
            double distance = cameraInputs.tagArray[i+3]; // meters
            double tagAngle = cameraInputs.tagArray[i+1]; // radians
            double robotAngle = getRawYaw().getRadians();

            // calculate offsets
            double angleToTag = tagAngle - robotAngle - PhysicalConstants.JETSON_OFFSET.getRotation().getZ();
            double xDistance = Math.cos(angleToTag) * distance - PhysicalConstants.JETSON_OFFSET.getX();
            double yDistance = Math.sin(angleToTag) * distance - PhysicalConstants.JETSON_OFFSET.getY();

            // add to temp map
            if (tempTagOffsetMap.containsKey(id)) { // if two cameras see one tag, average the values
                double[] oldArray = tempTagOffsetMap.get(id);
                tempTagOffsetMap.put(id, new double[] {
                    (xDistance + oldArray[0]) / 2, 
                    (yDistance + oldArray[1]) / 2, 
                    (angleToTag + oldArray[2]) / 2, 
                    ticksToKeep
                });
            } else {
                tempTagOffsetMap.put(id, new double[] {xDistance, yDistance, angleToTag, ticksToKeep});
            }

            // get pose from offset
            Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(id);
            accumulatedX += taglocation.getX() - xDistance;
            accumulatedY += taglocation.getY() - yDistance;

            tagCount++;
        }

        // combine both tag maps
        for (int key : tagOffsetMap.keySet()) {
            // if the id in the old array isn't in the temp array, add it to the temp array but subtract one from the counter
            if (tempTagOffsetMap.containsKey(key)) {
                continue;
            }

            double[] oldArray = tagOffsetMap.get(key);
            if (oldArray[3] <= 0) {
                continue;
            }
            tempTagOffsetMap.put(key, new double[] {
                oldArray[0],
                oldArray[1],
                oldArray[2],
                oldArray[3] - 1,
            });
        }
        tagOffsetMap = tempTagOffsetMap;

        if (tagCount == 0) {
            return new Pose2d();
        }
        return new Pose2d(accumulatedX / tagCount, accumulatedY / tagCount, getRawYaw());
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

    public double[] getOffsetFromSpecificTag(int id) {
        return tagOffsetMap.get(id);
    }
}