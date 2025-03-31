package frc.robot.subsystems.poseEstimator;


import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.PhysicalConstants;



public class CameraIOPhotonVision {
    PhotonCamera camera;
    PhotonPoseEstimator PoseEstimator;
    String CameraName;
    Optional<EstimatedRobotPose> EstimatedRobotPose;
    List<PhotonPipelineResult> EstimatorResult;
    String cameraPrefix;

    public CameraIOPhotonVision(
            PhotonCamera camera,
            String CameraName,
            Transform3d CameraTransform) {
        this.camera = camera;
        this.CameraName = CameraName;
        this.EstimatedRobotPose = Optional.empty();
        this.PoseEstimator = new PhotonPoseEstimator(PhysicalConstants.TagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        cameraPrefix = "Vision/" + CameraName + "/";
        System.out.println(CameraName + " Camera Innitilized");
        EstimatorResult = new ArrayList<>(); 
        EstimatorResult.add(new PhotonPipelineResult());
    }


    public void periodic(){
        EstimatorResult.clear();
        EstimatorResult = camera.getAllUnreadResults();
        if(EstimatorResult.size() > 0){
            EstimatedRobotPose = PoseEstimator.update(EstimatorResult.get(0));
            ArrayList<PhotonTrackedTarget> Tags = new ArrayList<PhotonTrackedTarget>();
            Pose3d Pose = UpdateVision();
    


            for (PhotonTrackedTarget tag : EstimatorResult.get(0).getTargets()) {
                Tags.add(tag);
            }


            Logger.recordOutput(cameraPrefix + "connected", camera.isConnected());
            Logger.recordOutput(cameraPrefix + "timestamp", EstimatorResult.get(0).getTimestampSeconds());
                
            if (Pose != null) {
                System.out.println("NOT NULLLLLLLL" + Pose);
                Logger.recordOutput(cameraPrefix + "VisionPose2d", Pose.toPose2d());
                Logger.recordOutput(cameraPrefix + "VisionPose3d", Pose);
            } else {
                System.out.println("NULLLLLLLL");
                Logger.recordOutput(cameraPrefix + "VisionPose2d", "null");
                Logger.recordOutput(cameraPrefix + "VisionPose3d", "null");
            }
                

            Logger.recordOutput(cameraPrefix + "NumTags", Tags.size());

        }


    }




    public Pose3d UpdateVision(){
        if (EstimatedRobotPose.isEmpty()){
            System.out.println("EstimatedRobotPose is empty");
            return null;
        }

        Pose3d pose = EstimatedRobotPose.get().estimatedPose;

        if (pose.getX() < 0.0 || pose.getX() >= PhysicalConstants.TagLayout.getFieldLength() ||
            pose.getY() < 0.0 || pose.getY() >= PhysicalConstants.TagLayout.getFieldWidth()) {
            System.out.println("Not in field");
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