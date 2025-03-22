package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIO cameraIO;
    private int jetsonLastPacketId;
    private int timesJetsonLastPacketIdWasStale;
    private int USB1LastPacketId;
    private int timesUSB1LastPacketIdWasStale;
    private HashMap<Integer, double[]> jetsonMap = new HashMap<Integer, double[]>();
    private HashMap<Integer, double[]> USB1Map = new HashMap<Integer, double[]>();
    private HashMap<Integer, double[]> combinedMap = new HashMap<Integer, double[]>();
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
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/fieldPosition", fieldPosition);
        Logger.recordOutput("outputs/poseEstimator/poses/odometryPoses/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        // vision // ! decrement hashmap values
        combinedMap = new HashMap<Integer, double[]>();
        if (cameraInputs.tagArray.length == 0) { // if packet is empty
            return;
        }
        int packetId = (int) cameraInputs.tagArray[0];
        int cameraId = (int) cameraInputs.tagArray[1];
        switch (cameraId) { // if packet is stale
            case 0: // jetson
                if (packetId == jetsonLastPacketId) {
                    timesJetsonLastPacketIdWasStale++;
                    System.out.println(timesJetsonLastPacketIdWasStale); // !
                } else {
                    timesJetsonLastPacketIdWasStale = 0;
                }
                if(timesJetsonLastPacketIdWasStale > 10){
                    return;
                }
                jetsonLastPacketId = packetId;
                break;
            case 1: // usb 1
                if (packetId == USB1LastPacketId) {
                    timesUSB1LastPacketIdWasStale++;
                    System.out.println(timesUSB1LastPacketIdWasStale); // !
                } else {
                    timesUSB1LastPacketIdWasStale = 0;
                }
                if(timesUSB1LastPacketIdWasStale > 10){
                    return;
                }

                USB1LastPacketId = packetId;
                break;
        }
        visionEstimate = updateVision();
        for(int key : combinedMap.keySet()){
            System.out.print(key + ", [");
            for(double d : combinedMap.get(key)){
                System.out.print(d + ", ");
            }
            System.out.println();
        }
        System.out.println("___________________________");
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public Pose2d updateVision() { // ! sleder's code keeps a tag in the networktables, even when it stops being seen
        // update jetson and usb 1 maps
        HashMap<Integer, HashMap<Integer, double[]>> cameraPacketMap = getCameraPacketMap();
        for(int key : cameraPacketMap.keySet()){
            HashMap<Integer, double[]> packetMap = cameraPacketMap.get(key);
            switch(key) {
                case 0: // jetson
                    HashMap<Integer, double[]> tempJetsonMap = cameraPacketMap.get(key);
                    for(int tagId : packetMap.keySet()){ // add all the new values from packetMap
                        tempJetsonMap.put(tagId, packetMap.get(tagId));
                    }
                    // transferring the old jetsonMap values
                    for (int jetsonKey : jetsonMap.keySet()) {
                        if (tempJetsonMap.containsKey(jetsonKey)) {
                            continue;
                        }

                        tempJetsonMap.put(jetsonKey, jetsonMap.get(jetsonKey));
                    }
                    jetsonMap = tempJetsonMap;
                    break;
                case 1: // usb 1
                    HashMap<Integer, double[]> tempUSB1Map = cameraPacketMap.get(key);
                    for(int tagId : packetMap.keySet()){ // add all the new values from packetMap
                        tempUSB1Map.put(tagId, packetMap.get(tagId));
                    }
                    // transferring the old usb 1 map values
                    for (int USB1Key : USB1Map.keySet()) {
                        if (tempUSB1Map.containsKey(USB1Key)) {
                            continue;
                        }

                        tempUSB1Map.put(USB1Key, USB1Map.get(USB1Key));
                    }
                    USB1Map = tempUSB1Map;
                    break;
            }
        }

        // combine jetson and usb 1 maps
        double accumulatedJetsonX = 0;
        double accumulatedJetsonY = 0;
        for(int jetsonKey : jetsonMap.keySet()){
            // log the jetsonMap
            Logger.recordOutput("outputs/poseEstimator/maps/jetsonMap/" + Integer.toString(jetsonKey), jetsonMap.get(jetsonKey));
            
            // add to jetson pose
            Pose2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(jetsonKey);
            accumulatedJetsonX += apriltagLocation.getX() - jetsonMap.get(jetsonKey)[2];
            accumulatedJetsonY += apriltagLocation.getY() - jetsonMap.get(jetsonKey)[3];
            
            // add to combinedMap
            combinedMap.put(jetsonKey, jetsonMap.get(jetsonKey));
        }
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/jetsonPose", new Pose2d(
            accumulatedJetsonX / jetsonMap.size(),
            accumulatedJetsonY / jetsonMap.size(),
            getRawYaw()
        ));

        double accumulatedUSB1X = 0;
        double accumulatedUSB1Y = 0;
        for(int USB1Key : USB1Map.keySet()){
            // log the usb 1 map
            Logger.recordOutput("outputs/poseEstimator/maps/USB1Map/" + Integer.toString(USB1Key), USB1Map.get(USB1Key));
            
            // add to usb 1 pose
            Pose2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(USB1Key);
            accumulatedUSB1X += apriltagLocation.getX() - USB1Map.get(USB1Key)[2];
            accumulatedUSB1Y += apriltagLocation.getY() - USB1Map.get(USB1Key)[3];

            // add to combinedMap
            if(combinedMap.containsKey(USB1Key)){ // average the values
                combinedMap.put(
                    USB1Key,
                    new double[] {
                        (combinedMap.get(USB1Key)[0] + USB1Map.get(USB1Key)[0]) / 2,
                        (combinedMap.get(USB1Key)[1] + USB1Map.get(USB1Key)[1]) / 2,
                        (combinedMap.get(USB1Key)[2] + USB1Map.get(USB1Key)[2]) / 2,
                        (combinedMap.get(USB1Key)[3] + USB1Map.get(USB1Key)[2]) / 2
                    }
                );
                continue;
            }
            combinedMap.put(USB1Key, USB1Map.get(USB1Key));
        }
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/USB1Pose", new Pose2d(
            accumulatedUSB1X / USB1Map.size(),
            accumulatedUSB1Y / USB1Map.size(),
            getRawYaw()
        ));

        // calculate robot pose
        double accumulatedX = 0;
        double accumulatedY = 0; // numtags = size
        for(int key : combinedMap.keySet()){
            Pose2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(key);
            accumulatedX += apriltagLocation.getX() - combinedMap.get(key)[2];
            accumulatedY += apriltagLocation.getY() - combinedMap.get(key)[3];
        }

        double size = combinedMap.size();
        return new Pose2d(accumulatedX / size, accumulatedY / size, getRawYaw());
    }

    public HashMap<Integer, HashMap<Integer, double[]>> getCameraPacketMap() {
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
        
        // get the packetMap
        HashMap<Integer, double[]> packetMap = new HashMap<Integer, double[]>();     
        for (int i = 2; i < cameraInputs.tagArray.length; i += 4) {
            // get networktables values
            int tagId = (int) cameraInputs.tagArray[i];
            double tagYaw = -cameraInputs.tagArray[i+1]; // radians, sleder's code does clockwise positive
            double tagPitch = cameraInputs.tagArray[i+2]; // radians
            double tagDistance = cameraInputs.tagArray[i+3]; // meters
            double robotYaw = getRawYaw().getRadians();

            // get distance from camera
            double pitchToTag = tagPitch;
            double hypotFromCamera = Math.cos(pitchToTag) * tagDistance;

            // get values from camera
            double yawFromCamera = tagYaw;
            double xFromCamera = Math.cos(yawFromCamera) * hypotFromCamera;
            double yFromCamera = Math.sin(yawFromCamera) * hypotFromCamera;

            // get values from robot center
            double xFromRobot = xFromCamera + cameraOffsetFromCenter.getX();
            double yFromRobot = yFromCamera + cameraOffsetFromCenter.getY();
            double hypotFromRobot = Math.hypot(xFromRobot, yFromRobot);
            double yawFromRobot = Math.acos(xFromRobot / hypotFromRobot);
            
            // get values from robot center relative to the field 
            double yawFromField = robotYaw + yawFromRobot;
            double xFromField = Math.cos(yawFromField) * hypotFromRobot;
            double yFromField = Math.sin(yawFromField) * hypotFromRobot;
            
            packetMap.put(tagId, new double[]{xFromRobot, yFromRobot, xFromField, yFromField});
        }

        HashMap<Integer, HashMap<Integer, double[]>> cameraPacketMap = new HashMap<Integer, HashMap<Integer, double[]>>();
        cameraPacketMap.put(cameraId, packetMap);
        return cameraPacketMap;
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

    public double[] getArrayFromSpecificTag(int id) {
        return combinedMap.get(id);
    }
}