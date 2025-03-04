package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.*;
import frc.robot.constants.*;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIOReal cameraIO;
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;
    private Pose2d fieldPosition = new Pose2d();

    private final Drive drive;

    public PoseEstimator(Drive drive){
        gyroIO = new GyroIOReal();
        cameraIO = new CameraIOReal();

        odometryEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d() // ! change position here for autos?
        );
        combinedEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d() // ! change position here for autos?
        );

        this.drive = drive;
    }

    @Override
    public void periodic(){
        // gyro
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);

        // camera
        cameraIO.updateInputs(cameraInputs);
        Logger.processInputs("poseEstimator/camera", cameraInputs);
        
        // pose
        Logger.recordOutput("arojaoijfoaf", drive.getModuleDeltas());
        Logger.recordOutput("boiajofjm", getYaw());

        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getYaw(),
            drive.getModulePositions()
        );
        visionEstimate = updateVisionEstimate();
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp()); 
        
        Logger.recordOutput("outputs/pose/fieldPosition", fieldPosition);

        Logger.recordOutput("outputs/pose/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/pose/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/pose/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public Pose2d updateVisionEstimate(){
        int tagCount = 0;
        double accumulatedX = 0;
        double accumulatedY = 0;
        for (int i = 1; i < cameraInputs.tagArray.length; i += 4) { // tags[0] is the packetId
            int id = (int) cameraInputs.tagArray[i];
            double distance = cameraInputs.tagArray[i+3]; // meters
            double tagAngle = cameraInputs.tagArray[i+1]; // radians
            double robotAngle = getYaw().getRadians();
            double angleToTag = tagAngle - robotAngle;
            double xDistance = Math.sin(angleToTag) * distance;
            double yDistance = Math.cos(angleToTag) * distance;

            Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(id);
            accumulatedX += taglocation.getX() - xDistance;
            accumulatedY += taglocation.getY() - yDistance;

            tagCount++;
        }

        if(tagCount > 0){
            return new Pose2d(accumulatedX / tagCount, accumulatedY / tagCount, getYaw());
        }

        return new Pose2d();
    }

    public void resetPosition(Pose2d pose){
        odometryEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
        combinedEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
    }

    public Pose2d getOdometryPose(){
        return odometryEstimator.getEstimatedPosition();
    }

    public Pose2d getVisionPose(){
        return visionEstimate;
    }

    public Pose2d getCombinedPose(){
        return combinedEstimator.getEstimatedPosition();
    }

    public Rotation2d getYaw(){
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(0) : 
            new Rotation2d(Math.PI)
        );
    }
}