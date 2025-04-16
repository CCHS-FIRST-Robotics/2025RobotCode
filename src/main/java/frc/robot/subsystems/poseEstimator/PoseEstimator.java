package frc.robot.subsystems.poseEstimator;



import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;




import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj.Timer;

import java.util.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.CameraIOPhotonVision.PoseDataEntry;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;


public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

 
    PhotonCamera FrontLeftCam, FrontRightCam, BackLeftCam, BackRightCam;
    List<PoseDataEntry> FrontLeftPoses, FrontRightPose, BackLeftPose, BackRightPose;
    CameraIOPhotonVision FrontLeftEstimator, FrontRightEstimator,BackLeftEstimator ,BackRightEstimator;
    

    private Pose2d fieldPosition = new Pose2d();
    private final SwerveDrivePoseEstimator odometryEstimator;
    private final SwerveDrivePoseEstimator combinedEstimator;
    

    private final Drive drive;


    public PoseEstimator(
        GyroIO gyroIO, 
        Drive drive
    ) {
        this.gyroIO = gyroIO;

        FrontLeftCam = new PhotonCamera("Cam");
        FrontRightCam = new PhotonCamera("Cam2");
        BackLeftCam = new PhotonCamera("LeftRear");
        BackRightCam = new PhotonCamera("RightRear");
    
        FrontLeftEstimator = new CameraIOPhotonVision(FrontLeftCam, FrontLeftCam.getName(), PhysicalConstants.FrontLeftCamToCenter);
        FrontRightEstimator = new CameraIOPhotonVision(FrontRightCam, FrontRightCam.getName(), PhysicalConstants.FrontRightCamToCenter);
        BackLeftEstimator = new CameraIOPhotonVision(BackLeftCam, BackLeftCam.getName(), PhysicalConstants.BackLeftCamToCenter);
        BackRightEstimator = new CameraIOPhotonVision(BackRightCam, BackRightCam.getName(), PhysicalConstants.BackRightCamToCenter);
        
        

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
            new Pose2d(),
             VirtualConstants.DriveStdDevs,
            VirtualConstants.SingleTagStdDevs
        );
            
        this.drive = drive;
    }

    @Override
    public void periodic() {
        FrontLeftEstimator.periodic();
        FrontRightEstimator.periodic();
        BackLeftEstimator.periodic();
        BackRightEstimator.periodic();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);
        FrontLeftPoses = FrontLeftEstimator.getVisionPoses();
        FrontRightPose = FrontRightEstimator.getVisionPoses();
        BackLeftPose = BackLeftEstimator.getVisionPoses();
        BackRightPose = BackRightEstimator.getVisionPoses();
        

        if (FrontLeftPoses != null) {
            for (PoseDataEntry pose : FrontLeftPoses){
            combinedEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }

        if (FrontRightPose != null) {
            for (PoseDataEntry pose : FrontRightPose){
                combinedEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }
        
        if (BackLeftPose != null) {
            for (PoseDataEntry pose : BackLeftPose){
                combinedEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }

        if (BackRightPose != null) {
            for (PoseDataEntry pose : BackRightPose){
                combinedEstimator.addVisionMeasurement(pose.getRobotPose().toPose2d(), pose.getTimestamp(), pose.getStandardDeviation());
            }
        }

        

        
        

        
        // odometry
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
       
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());

        

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
    private Pose2d getCombinedPose() {
        return combinedEstimator.getEstimatedPosition();
    }

    public Pose2d getPose() {
        return getOdometryPose();
    }

    public Rotation2d getRawYaw() {
        return fieldPosition.getRotation();
    }

}
