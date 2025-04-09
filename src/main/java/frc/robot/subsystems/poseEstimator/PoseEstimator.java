package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;


import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj.Timer;

import java.util.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;


public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    //private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

 
    PhotonCamera FrontLeftCam, FrontRightCam, BackLeftCam, BackRightCam;
    
    EstimatedRobotPose FrontLeftPose, FrontRightPose, BackLeftPose, BackRightPose;
    PhotonPipelineResult FrontLeftEstimatorResult, FrontRightEstimatorResult ,BackLeftEstimatorResult ,BackRightEstimatorResult;
    CameraIOPhotonVision FrontLeftEstimator, FrontRightEstimator,BackLeftEstimator ,BackRightEstimator;
    List<Pose3d> poseList;

    private Pose2d fieldPosition = new Pose2d();
    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private final Drive drive;


    public PoseEstimator(
        GyroIO gyroIO, 
        Drive drive
    ) {
        this.gyroIO = gyroIO;

        FrontLeftCam = new PhotonCamera("Cam");
        //FrontRightCam = new PhotonCamera("RightFront");
        //BackLeftCam = new PhotonCamera("LeftRear");
        //BackRightCam = new PhotonCamera("RightRear");
    
        FrontLeftEstimator = new CameraIOPhotonVision(FrontLeftCam, FrontLeftCam.getName(), PhysicalConstants.FrontLeftCamToCenter);
        //FrontRightEstimator = new CameraIOPhotonVision(FrontRightCam, FrontRightCam.getName(), PhysicalConstants.FrontRightCamToCenter);
        //BackLeftEstimator = new CameraIOPhotonVision(BackLeftCam, BackLeftCam.getName(), PhysicalConstants.BackLeftCamToCenter);
        //BackRightEstimator = new CameraIOPhotonVision(BackRightCam, BackRightCam.getName(), PhysicalConstants.BackRightCamToCenter);
        
        poseList = new ArrayList<>(4);

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
        FrontLeftEstimator.periodic();
        // gyroIO.updateInputs(gyroInputs);
        // Logger.processInputs("poseEstimator/gyro", gyroInputs);
        FrontLeftPose = FrontLeftEstimator.getEstimatedRobotPose();
        // FrontRightPose = FrontRightEstimator.getEstimatedRobotPose().get();
        // BackLeftPose = BackLeftEstimator.getEstimatedRobotPose().get();
        // BackRightPose = BackRightEstimator.getEstimatedRobotPose().get();
        

        if (FrontLeftPose != null) {
            combinedEstimator.addVisionMeasurement(FrontLeftPose.estimatedPose.toPose2d(), FrontLeftPose.timestampSeconds, FrontLeftEstimator.getEstimationStdDevs());
          }

        // if (FrontRightPose != null) {
        //     poseList.add(FrontRightPose);
        //   }
        
        // if (BackLeftPose != null) {
        //     poseList.add(BackLeftPose);
        //   }

        // if (BackRightPose != null) {
        //     poseList.add(BackRightPose);
        //   }

        

        
        

        
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
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
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
        return fieldPosition.getRotation();
    }

}
