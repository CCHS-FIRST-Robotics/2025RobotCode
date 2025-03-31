package frc.robot.subsystems.poseEstimator;


import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;



public class CameraIOPhotonVision implements Subsystem{
    PhotonCamera camera;
    PhotonPoseEstimator PoseEstimator;
    String CameraName;
    Optional<EstimatedRobotPose> EstimatedRobotPose;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPipelineResult EstimatorResult;
    String cameraPrefix;
    

    
    public CameraIOPhotonVision(
        PhotonCamera camera,
        String CameraName,
        Transform3d CameraTransform
    ){
        this.camera = camera;
        this.CameraName = CameraName;
        this.EstimatedRobotPose =  Optional.empty();
        this.EstimatorResult = new PhotonPipelineResult();
        this.PoseEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraTransform);
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        cameraPrefix = "Vision/" + CameraName + "/";
        
        

    }



    @Override
    public void periodic(){
        EstimatorResult = camera.getLatestResult();
        EstimatedRobotPose = PoseEstimator.update(EstimatorResult);
        ArrayList<PhotonTrackedTarget> Tags = new ArrayList<PhotonTrackedTarget>();
        Pose3d Pose = UpdateVision();

        for (PhotonTrackedTarget tag : EstimatorResult.getTargets()) {
            Tags.add(tag);
        }


        Logger.recordOutput(cameraPrefix + "connected", camera.isConnected());
        Logger.recordOutput(cameraPrefix + "timestamp", EstimatorResult.getTimestampSeconds());
            
        if (Pose != null) {
            Logger.recordOutput(cameraPrefix + "VisionPose2d", Pose.toPose2d());
            Logger.recordOutput(cameraPrefix + "VisionPose3d", Pose);
        } else {
            Logger.recordOutput(cameraPrefix + "VisionPose2d", "null");
            Logger.recordOutput(cameraPrefix + "VisionPose3d", "null");
        }
            

        Logger.recordOutput(cameraPrefix + "NumTags", Tags.size());


    }




    public Pose3d UpdateVision(){
        if (EstimatedRobotPose.isEmpty()){
            return null;
        }

        Pose3d pose = EstimatedRobotPose.get().estimatedPose;

        if (pose.getX() < 0.0 || pose.getX() >= aprilTagFieldLayout.getFieldLength() ||
            pose.getY() < 0.0 || pose.getY() >= aprilTagFieldLayout.getFieldWidth()) {
            
            return null;
        }

        return pose;
    }




    public Pose3d GetPoseEstimation(){
        Pose3d Pose = UpdateVision();
        if(Pose != null){
            return Pose;
        }
        return null;
        
        
    }

    
}