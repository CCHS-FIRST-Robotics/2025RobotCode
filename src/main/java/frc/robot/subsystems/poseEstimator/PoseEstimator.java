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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;

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

        PhotonCamera FrontLeftCam = new PhotonCamera("LeftFront");
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
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1), //State standard deviations (x, y, heading)  tune
            VecBuilder.fill(0.1, 0.1, 0.1)  // multi tag Vision measurement standard deviations (x, y, heading)
            
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

        if (FrontLeftUpdate.isPresent()) {
            EstimatedRobotPose pose = FrontLeftUpdate.get();
            Logger.recordOutput("poseEstimator/FrontLeftPose", pose.estimatedPose);
            Logger.recordOutput("poseEstimator/FrontLeftTimestamp", pose.timestampSeconds);
            Logger.recordOutput("poseEstimator/FrontLeftTargets", pose.targetsUsed.toString());
        }
        
        if (FrontRightUpdate.isPresent()) {
            EstimatedRobotPose pose = FrontRightUpdate.get();
            Logger.recordOutput("poseEstimator/FrontRightPose", pose.estimatedPose);
            Logger.recordOutput("poseEstimator/FrontRightTimestamp", pose.timestampSeconds);
            Logger.recordOutput("poseEstimator/FrontRightTargets", pose.targetsUsed.toString());
        }
        
        if (BackLeftUpdate.isPresent()) {
            EstimatedRobotPose pose = BackLeftUpdate.get();
            Logger.recordOutput("poseEstimator/BackLeftPose", pose.estimatedPose);
            Logger.recordOutput("poseEstimator/BackLeftTimestamp", pose.timestampSeconds);
            Logger.recordOutput("poseEstimator/BackLeftTargets", pose.targetsUsed.toString());
        }
        
        if (BackRightUpdate.isPresent()) {
            EstimatedRobotPose pose = BackRightUpdate.get();
            Logger.recordOutput("poseEstimator/BackRightPose", pose.estimatedPose);
            Logger.recordOutput("poseEstimator/BackRightTimestamp", pose.timestampSeconds);
            Logger.recordOutput("poseEstimator/BackRightTargets", pose.targetsUsed.toString());
        }
        
        

    }



    public void updateVision() {
        List<Pose2d> poseList = new ArrayList<>(10);

        final Optional<EstimatedRobotPose> optionalFREstimatedPoseRight = FrontLeftEstimator.update(FrontLeftEstimatorResult);
            if (UseData(optionalFREstimatedPoseRight)) {
                final EstimatedRobotPose estimatedPose = optionalFREstimatedPoseRight.get();          
                combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.VISION_STDS);
                poseList.add(estimatedPose.estimatedPose.toPose2d());
                
    }
        final Optional<EstimatedRobotPose> optionalFLEstimatedPoseRight = FrontRightEstimator.update(FrontRightEstimatorResult);
            if (UseData(optionalFLEstimatedPoseRight)) {
                final EstimatedRobotPose estimatedPose = optionalFLEstimatedPoseRight.get();       
                combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.VISION_STDS);   
                poseList.add(estimatedPose.estimatedPose.toPose2d());
    }

    final Optional<EstimatedRobotPose> optionalBREstimatedPoseRight = BackLeftEstimator.update(BackLeftEstimatorResult);
        if (UseData(optionalBREstimatedPoseRight)) {
            final EstimatedRobotPose estimatedPose = optionalBREstimatedPoseRight.get(); 
            combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.VISION_STDS);         
            poseList.add(estimatedPose.estimatedPose.toPose2d());
    }

    final Optional<EstimatedRobotPose> optionalBLEstimatedPoseRight = BackRightEstimator.update(BackRightEstimatorResult);
        if (UseData(optionalBREstimatedPoseRight)) {
            final EstimatedRobotPose estimatedPose = optionalBLEstimatedPoseRight.get();   
            combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.VISION_STDS);       
            poseList.add(estimatedPose.estimatedPose.toPose2d());
    double sumX = 0; // Declare and initialize outside the loop
    double sumY = 0;
    double sumRotationRadians = 0;
    
    for (Pose2d pose : poseList) {
        sumX += pose.getX(); // Use += inside the loop
        sumY += pose.getY();
        sumRotationRadians += pose.getRotation().getRadians();
    }
    
    double averageX = sumX / poseList.size();
    double averageY = sumY / poseList.size();
    double averageRotationRadians = sumRotationRadians / poseList.size();
    visionEstimate = new Pose2d(averageX, averageY, new Rotation2d(averageRotationRadians));
}
    

}
        
        


    public Boolean UseData(Optional<EstimatedRobotPose> robotpose){
        Boolean use = true;
        if(robotpose.isEmpty()){ // if empty
            return false; // dont want to continue without any values
        }
        Pose3d estimatedPose = robotpose.get().estimatedPose;
        
        if (estimatedPose.getX() < 0.0 || estimatedPose.getX() >= aprilTagFieldLayout.getFieldLength() // if not in bounds of field
        || estimatedPose.getY() < 0.0 || estimatedPose.getY() >= aprilTagFieldLayout.getFieldWidth()) {   
            use = false;
        }

        if (Math.abs(Math.toDegrees(estimatedPose.getRotation().getZ()) - getRawYaw().getRadians()) > 4){
            use = false;
        }
        
        return use;
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

}