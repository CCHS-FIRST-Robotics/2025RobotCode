package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.Drive;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIOReal cameraIO;
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private final SwerveDrivePoseEstimator odometryEstimator;
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
            new Pose2d(1, 1, new Rotation2d()) // ! change position here for autos?
        );
        combinedEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d(1, 1, new Rotation2d()) // ! change position here for autos?
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
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            gyroInputs.connected ? getYaw() : fieldPosition.getRotation(),
            drive.getModulePositions()
        );

        Pose2d visionEstimate = getCameraPose();
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            gyroInputs.connected ? getYaw() : fieldPosition.getRotation(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp()); 
        
        Logger.recordOutput("outputs/pose/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/pose/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/pose/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public void resetPosition(Pose2d pose){
        odometryEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
        combinedEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
    }

    public Pose2d getCameraPose(){ // ! sigh... shouldn't this also run on the jetson?
        int tagCount = 0;
        double estimatedX = 0;
        double estimatedY = 0;
        for (int i = 1; i < cameraInputs.tagArray.length; i += 4) { // ! tags[0] is the something
            int id = (int) cameraInputs.tagArray[i];
            double distance = cameraInputs.tagArray[i+3]; // meters
            double tagAngle = cameraInputs.tagArray[i+1]; // radians
            double robotAngle = getYaw().getRadians();
            double angleToTag = tagAngle - robotAngle;
            double dx = Math.sin(angleToTag) * distance;
            double dy = Math.cos(angleToTag) * distance;

            Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(id);
            estimatedX += taglocation.getX() - dx;
            estimatedY += taglocation.getY() - dy;

            tagCount++;
        }
        estimatedX /= tagCount;
        estimatedY /= tagCount;

        return new Pose2d(estimatedX, estimatedY, getYaw());
    }

    public Pose2d getPose(){
        return odometryEstimator.getEstimatedPosition();
    }

    public Rotation2d getYaw() {
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : getPose().getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(0) : 
            new Rotation2d(Math.PI)
        );
    }
}