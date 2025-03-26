package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera cam;
    Transform3d robotToCam;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonPipelineResult LastFrontLeftEstimatorResult, LastFrontRightEstimatorResult , LastBackLeftEstimatorResult ,LastBackRightEstimatorResult;
    private PhotonCamera FrontLeftCam, FrontRightCam, BackLeftCam, BackRightCam;
    private PhotonPoseEstimator FrontLeftEstimator, FrontRightEstimator, BackLeftEstimator, BackRightEstimator;
    Optional<EstimatedRobotPose> FrontLeftUpdate, FrontRightUpdate, BackLeftUpdate, BackRightUpdate;
    private AprilTagFieldLayout FieldLayout;
    PhotonPipelineResult FrontLeftEstimatorResult, FrontRightEstimatorResult ,BackLeftEstimatorResult ,BackRightEstimatorResult;



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

        FrontLeftCam = new PhotonCamera("LeftFront");
        FrontRightCam = new PhotonCamera("RightFront");
        BackLeftCam = new PhotonCamera("LeftRear");
        BackRightCam = new PhotonCamera("RightRear");

        FrontLeftUpdate = Optional.empty();
        FrontRightUpdate = Optional.empty();
        BackLeftUpdate = Optional.empty();
        BackRightUpdate = Optional.empty();


        FrontLeftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhysicalConstants.FrontLeftCamToCenter);
        FrontRightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhysicalConstants.FrontRightCamToCenter);
        BackRightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhysicalConstants.BackRightCamToCenter);
        BackLeftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhysicalConstants.BackLeftCamToCenter);

        // When only one tag can be seen
        FrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        FrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        BackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        BackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        FrontLeftEstimatorResult = new PhotonPipelineResult();
        FrontRightEstimatorResult = new PhotonPipelineResult();
        BackLeftEstimatorResult = new PhotonPipelineResult();
        BackRightEstimatorResult = new PhotonPipelineResult();


       



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
        gyroIO.updateInputs(gyroInputs);
    
        Logger.processInputs("poseEstimator/gyro", gyroInputs);
        FrontLeftUpdate = FrontLeftEstimator.update(LastFrontLeftEstimatorResult);
        FrontRightUpdate = FrontRightEstimator.update(LastFrontRightEstimatorResult);
        BackLeftUpdate = BackLeftEstimator.update(LastBackLeftEstimatorResult);
        BackRightUpdate = BackRightEstimator.update(LastBackRightEstimatorResult);

        LastFrontLeftEstimatorResult =  FrontLeftCam.getLatestResult();
        LastFrontRightEstimatorResult = FrontRightCam.getLatestResult();
        LastBackLeftEstimatorResult = BackLeftCam.getLatestResult();
        LastBackRightEstimatorResult = BackRightCam.getLatestResult();
    
        
        // odometry
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        
        updateVision();
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());

        //! ask alex
        try{
            if(FrontLeftUpdate.isPresent()){
                Logger.recordOutput("poseEstimator/FrontLeftCameraUpdate", EstimatedRobotPose.struct ,FrontLeftUpdate.get());
                
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front left estimator had no update to get");
        }

        try{
            if(FrontRightUpdate.isPresent()){
                Logger.recordOutput("poseEstimator/FrontrightCameraUpdate", FrontRightUpdate.get());
                
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front right estimator had no update to get");
        }
        
         try{
            if(BackRightUpdate.isPresent()){
                Logger.recordOutput("poseEstimator/FrontLeftCameraUpdate", BackLeftUpdate.get());
                
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back left estimator had no update to get");
        }
         try{
            if(BackRightUpdate.isPresent()){
                Logger.recordOutput("poseEstimator/FrontLeftCameraUpdate", BackRightUpdate.get());
                
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back right estimator had no update to get");
    }

    }



    public void updateVision() {
        
            final Optional<EstimatedRobotPose> optionalFREstimatedPoseRight = FrontLeftEstimator.update(FrontLeftEstimatorResult);
    if (optionalFREstimatedPoseRight.isPresent()) {
        final EstimatedRobotPose estimatedPose = optionalFREstimatedPoseRight.get();          
        combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }
    final Optional<EstimatedRobotPose> optionalFLEstimatedPoseRight = FrontRightEstimator.update(FrontRightEstimatorResult);
    if (optionalFLEstimatedPoseRight.isPresent()) {
        final EstimatedRobotPose estimatedPose = optionalFLEstimatedPoseRight.get();       
        combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);   
        //combinedEstimator.updateVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }

    final Optional<EstimatedRobotPose> optionalBREstimatedPoseRight = BackLeftEstimator.update(BackLeftEstimatorResult);
    if (optionalBREstimatedPoseRight.isPresent()) {
        final EstimatedRobotPose estimatedPose = optionalBREstimatedPoseRight.get(); 
        combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);         
        //combinedEstimator.updateVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }

    final Optional<EstimatedRobotPose> optionalBLEstimatedPoseRight = BackRightEstimator.update(BackRightEstimatorResult);
    if (optionalBLEstimatedPoseRight.isPresent()) {
        final EstimatedRobotPose estimatedPose = optionalBLEstimatedPoseRight.get();   
        combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);       
        //combinedEstimator.updateVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }


}
        
        

    public void resetPosition(Pose2d pose) {
        odometryEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
        combinedEstimator.resetPosition(pose.getRotation(), drive.getModulePositions(), pose); // ! doesn't have stddevs of anything
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

}