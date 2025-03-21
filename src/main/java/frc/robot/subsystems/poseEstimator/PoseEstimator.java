package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIO cameraIO;
    private int jetsonLastPacketId;
    private int USB1LastPacketId;
    private HashMap<Integer, double[]> jetsonTagOffsetMap = new HashMap<Integer, double[]>();
    private HashMap<Integer, double[]> USB1TagOffsetMap = new HashMap<Integer, double[]>();
    private HashMap<Integer, double[]> combinedTagOffsetMap = new HashMap<Integer, double[]>();
    private final int ticksToKeep = 10;
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
        // inputs
        gyroIO.updateInputs(gyroInputs);
        cameraIO.updateInputs(cameraInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);
        Logger.processInputs("poseEstimator/camera", cameraInputs);
        
        // odometry
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
        odometryEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        Logger.recordOutput("outputs/poseEstimator/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        // vision

        // if the packet is empty or stale, return 
        if (cameraInputs.tagArray.length == 0) {
            return;
        }
        int packetId = (int) cameraInputs.tagArray[0];
        int cameraId = (int) cameraInputs.tagArray[1];
        switch (cameraId) {
            case 0: 
                if(packetId == jetsonLastPacketId){ // if stale
                    return;
                }
                jetsonLastPacketId = packetId;
                break;
            case 1: 
                if(packetId == USB1LastPacketId){ // if stale
                    return;
                }
                USB1LastPacketId = packetId;
                break;
        }
        System.out.println("UPDATING");
        visionEstimate = updateVisionEstimate();

        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp()); 

        Logger.recordOutput("outputs/poseEstimator/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    /** 
     * this gets a hashmap per packet of the estimated offsets to each tag
     */
    public HashMap<Integer, HashMap<Integer, double[]>> getCameraPacketTagOffsetMap() {
        // get the camera offset
        int cameraId = (int) cameraInputs.tagArray[1];
        Pose2d cameraOffsetFromCenter = new Pose2d();
        switch (cameraId) {
            case 0: 
                cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
                break;
            case 1: 
                cameraOffsetFromCenter = PhysicalConstants.USB_CAMERA_1_OFFSET;
                break;
        }
        
        // get the packetOffsetMap
        HashMap<Integer, double[]> packetTagOffsetMap = new HashMap<Integer, double[]>();// tag id, camera id and offset array        
        for (int i = 2; i < cameraInputs.tagArray.length; i += 4) {
            // get networktables values
            int tagId = (int) cameraInputs.tagArray[i];
            double tagYaw = -cameraInputs.tagArray[i+1]; // radians, sleder's code does clockwise positive
            double tagPitch = cameraInputs.tagArray[i+2]; // radians
            double tagDistance = cameraInputs.tagArray[i+3]; // meters
            double robotYaw = getRawYaw().getRadians();

            // get 2DDistance
            double pitchToTag = tagPitch;
            double twoDimensionalDistance = Math.cos(pitchToTag) * tagDistance;

            // get camera adjusted values
            double yawWithoutCameraOffset = tagYaw;
            double xWithoutCameraOffset = Math.cos(yawWithoutCameraOffset) * twoDimensionalDistance;
            double yWithoutCameraOffset = Math.sin(yawWithoutCameraOffset) * twoDimensionalDistance;

            // add the camera xy offset
            double xWithRobotRotationOffset = xWithoutCameraOffset + cameraOffsetFromCenter.getX();
            double yWithRobotRotationOffset = yWithoutCameraOffset + cameraOffsetFromCenter.getY();
            double hypotWithRobotRotationOffset = Math.hypot(xWithRobotRotationOffset, yWithRobotRotationOffset);
            double yawWithRobotRotationOffset = Math.acos(xWithRobotRotationOffset / hypotWithRobotRotationOffset); // good value
            
            // get the actual 
            double yawToTagFromRobotCenterLine = robotYaw - yawWithRobotRotationOffset;
            double xToTagFromRobotCenterLine = Math.cos(yawToTagFromRobotCenterLine) * hypotWithRobotRotationOffset; // good value
            double yToTagFromRobotCenterLine = Math.sin(yawToTagFromRobotCenterLine) * hypotWithRobotRotationOffset; // good value
            
            packetTagOffsetMap.put(tagId, new double[]{xToTagFromRobotCenterLine, yToTagFromRobotCenterLine, getRawYaw().getRadians(), ticksToKeep});
        }

        HashMap<Integer, HashMap<Integer, double[]>> cameraPacketTagOffsetMap = new HashMap<Integer, HashMap<Integer, double[]>>();
        cameraPacketTagOffsetMap.put(cameraId, packetTagOffsetMap);
        return cameraPacketTagOffsetMap;
    }

    public Pose2d updateVisionEstimate() {
        // update the jetson and USB camera offset maps
        HashMap<Integer, HashMap<Integer, double[]>> cameraPacketTagOffsetMap = getCameraPacketTagOffsetMap();
        for(int key : cameraPacketTagOffsetMap.keySet()){
            HashMap<Integer, double[]> packetTagOffsetMap = cameraPacketTagOffsetMap.get(key);
            switch(key){
                case 0: // jetson
                    for(int key1 : packetTagOffsetMap.keySet()){ // add all the new values
                        jetsonTagOffsetMap.put(key1, packetTagOffsetMap.get(key1));
                    }
                    // transferring the cameraPacketTagoffsetmap to the jetsonoffsetmap
                    for (int jetsonKey : jetsonTagOffsetMap.keySet()){ // go through the old values, see if they're overdue
                        if(packetTagOffsetMap.containsKey(jetsonKey)){ // if it has it, just replace it
                            continue;
                        }

                        if (jetsonTagOffsetMap.get(jetsonKey)[3] <= 0){
                            continue;
                        }
                        jetsonTagOffsetMap.put(jetsonKey, new double[]{
                            jetsonTagOffsetMap.get(jetsonKey)[0],
                            jetsonTagOffsetMap.get(jetsonKey)[1],
                            jetsonTagOffsetMap.get(jetsonKey)[2],
                            jetsonTagOffsetMap.get(jetsonKey)[3] - 1
                        });
                    }
                    break;
                case 1: // usb 1
                    for(int key1 : packetTagOffsetMap.keySet()){ // add all the new values
                        jetsonTagOffsetMap.put(key1, packetTagOffsetMap.get(key1));
                    }
                    // transferring the cameraPacketTagoffsetmap to the jetsonoffsetmap
                    for (int jetsonKey : jetsonTagOffsetMap.keySet()){ // go through the old values, see if they're overdue
                        if(packetTagOffsetMap.containsKey(jetsonKey)){ // if it has it, just replace it
                            continue;
                        }

                        if (jetsonTagOffsetMap.get(jetsonKey)[3] <= 0){
                            continue;
                        }
                        jetsonTagOffsetMap.put(jetsonKey, new double[]{
                            jetsonTagOffsetMap.get(jetsonKey)[0],
                            jetsonTagOffsetMap.get(jetsonKey)[1],
                            jetsonTagOffsetMap.get(jetsonKey)[2],
                            jetsonTagOffsetMap.get(jetsonKey)[3] - 1
                        });
                    }
                    break;
            }
        }

        for(int USB1Key : USB1TagOffsetMap.keySet()){
            if(jetsonTagOffsetMap.containsKey(USB1Key)){ // if they both have an apriltag, put the usb1 into the jetson
                jetsonTagOffsetMap.put(
                    USB1Key,
                    new double[] {
                        (jetsonTagOffsetMap.get(USB1Key)[0] + USB1TagOffsetMap.get(USB1Key)[0]) / 2,
                        (jetsonTagOffsetMap.get(USB1Key)[1] + USB1TagOffsetMap.get(USB1Key)[1]) / 2,
                        (jetsonTagOffsetMap.get(USB1Key)[2] + USB1TagOffsetMap.get(USB1Key)[2]) / 2
                    }
                );
            }else{ // if only the usb has it, just add it to the jetson
                jetsonTagOffsetMap.put(
                    USB1Key,
                    new double[] {
                        USB1TagOffsetMap.get(USB1Key)[0],
                        USB1TagOffsetMap.get(USB1Key)[1],
                        USB1TagOffsetMap.get(USB1Key)[2]
                    }
                );
            }
        }
        combinedTagOffsetMap = jetsonTagOffsetMap;

        int accumulatedX = 0;
        int accumulatedY = 0; // numtags = size
        for(int key : combinedTagOffsetMap.keySet()){
            Translation2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(key);
            accumulatedX += apriltagLocation.getX() - combinedTagOffsetMap.get(key)[0];
            accumulatedY += apriltagLocation.getY() - combinedTagOffsetMap.get(key)[1];
        }

        int size = combinedTagOffsetMap.size();
        return new Pose2d(accumulatedX / size, accumulatedY / size, getRawYaw());
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

    public double[] getOffsetFromSpecificTag(int id) {
        return combinedTagOffsetMap.get(id);
    }
}