package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
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
    Optional<EstimatedRobotPose> FrontLeftPose, FrontRightPose, BackLeftPose, BackRightPose;
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

        FrontLeftPose = Optional.empty();
        FrontRightPose = Optional.empty();
        BackLeftPose = Optional.empty();
        BackRightPose = Optional.empty();


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
            VecBuilder.fill(0.3, 0.3, 0.1), //State standard deviations (x, y, heading)  tune
            VecBuilder.fill(0.5, 0.5, 0.5)  // multi tag Vision measurement standard deviations (x, y, heading)
            
        );

        this.drive = drive;
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);

        FrontLeftEstimatorResult = FrontLeftCam.getLatestResult();
        FrontRightEstimatorResult = FrontRightCam.getLatestResult();
        BackLeftEstimatorResult = BackLeftCam.getLatestResult();
        BackRightEstimatorResult = BackRightCam.getLatestResult();

        FrontLeftPose = FrontLeftEstimator.update(LastFrontLeftEstimatorResult);
        FrontRightPose = FrontRightEstimator.update(LastFrontRightEstimatorResult);
        BackLeftPose = BackLeftEstimator.update(LastBackLeftEstimatorResult);
        BackRightPose = BackRightEstimator.update(LastBackRightEstimatorResult);
        
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
            if ((optionalFREstimatedPoseRight.isPresent())) {
                final EstimatedRobotPose estimatedPose = optionalFREstimatedPoseRight.get();          
                combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.MULTI_TAG_VISION_STDS);
                poseList.add(estimatedPose.estimatedPose.toPose2d());
            }
                
    
        final Optional<EstimatedRobotPose> optionalFLEstimatedPoseRight = FrontRightEstimator.update(FrontRightEstimatorResult);
                if (optionalFLEstimatedPoseRight.isPresent()) {
                        final EstimatedRobotPose estimatedPose = optionalFLEstimatedPoseRight.get();       
                        combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.MULTI_TAG_VISION_STDS);   
                        poseList.add(estimatedPose.estimatedPose.toPose2d());
            }

        final Optional<EstimatedRobotPose> optionalBREstimatedPoseRight = BackLeftEstimator.update(BackLeftEstimatorResult);
            if (optionalBREstimatedPoseRight.isPresent()) {
                final EstimatedRobotPose estimatedPose = optionalBREstimatedPoseRight.get(); 
                combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.MULTI_TAG_VISION_STDS);         
                poseList.add(estimatedPose.estimatedPose.toPose2d());
        }

        final Optional<EstimatedRobotPose> optionalBLEstimatedPoseRight = BackRightEstimator.update(BackRightEstimatorResult);
            if (optionalBREstimatedPoseRight.isPresent()) {
                final EstimatedRobotPose estimatedPose = optionalBLEstimatedPoseRight.get();   
                combinedEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, VirtualConstants.MULTI_TAG_VISION_STDS);       
                poseList.add(estimatedPose.estimatedPose.toPose2d());
    double sumX = 0; 
    double sumY = 0;
    double sumRotationRadians = 0;
    
    for (Pose2d pose : poseList) {
        sumX += pose.getX(); 
        sumY += pose.getY();
        sumRotationRadians += pose.getRotation().getRadians();
    }
    
    double averageX = sumX / poseList.size();
    double averageY = sumY / poseList.size();
    double averageRotationRadians = sumRotationRadians / poseList.size();
    visionEstimate = new Pose2d(averageX, averageY, new Rotation2d(averageRotationRadians));
}
    

}
        
        

public boolean useData(List<Pose2d> poseList) {
    boolean use = true;
    Transform2d maxAllowedErr = new Transform2d(Meters.of(0.5), Meters.of(0.5), new Rotation2d(Units.degreesToRadians(10)));
    List<Pose2d> checkedPoseList = new ArrayList<>(4);

    for (Pose2d pose : poseList) {
        if (pose.getX() < 0.0 || pose.getX() >= aprilTagFieldLayout.getFieldLength() ||
            pose.getY() < 0.0 || pose.getY() >= aprilTagFieldLayout.getFieldWidth()) {
            use = false;
            break; 
        }

        if (Math.abs(Math.toDegrees(pose.getRotation().getDegrees()) - Math.toDegrees(getRawYaw().getDegrees())) >= 4) {
            use = false;
            break; 
        }

        checkedPoseList.add(pose); // Add the pose to the checked list
    }

    if (!use) {
        return false; // If any pose failed the initial checks, reject all
    }

    // Check for pose discrepancies within the list
    for (int i = 0; i < checkedPoseList.size(); i++) {
        for (int j = i + 1; j < checkedPoseList.size(); j++) {
            Transform2d err = checkedPoseList.get(i).minus(checkedPoseList.get(j));

            if (Math.abs(err.getX()) > maxAllowedErr.getX() ||
                Math.abs(err.getY()) > maxAllowedErr.getY() ||
                Math.abs(Math.toDegrees(err.getRotation().getDegrees())) > Math.toDegrees(maxAllowedErr.getRotation().getDegrees())) {
                return false; // Reject all poses if any discrepancy exceeds the threshold
            }
        }
    }

    return true;
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
