package frc.robot.subsystems.poseEstimator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();
    private final int ticksToKeep = 10;
    private final int numCameras = 5;
    private final int[] cameraLastPacketIds;
    private final int[] timesLastCameraPacketWasStale;
    private final ArrayList<HashMap<Integer, double[]>> cameraMaps;
    private HashMap<Integer, double[]> combinedMap = new HashMap<Integer, double[]>();

    private Pose2d fieldPosition = new Pose2d();
    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private final Drive drive;

    public PoseEstimator(
        GyroIO gyroIO, 
        CameraIO cameraIO, 
        Drive drive
    ) {
        this.gyroIO = gyroIO;
        this.cameraIO = cameraIO;

        cameraLastPacketIds = new int[numCameras];
        timesLastCameraPacketWasStale = new int[numCameras];
        cameraMaps = new ArrayList<HashMap<Integer, double[]>>();
        for(int i = 0; i < numCameras; i++){
            cameraMaps.add(new HashMap<Integer, double[]>());
        }

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

        // vision
        combinedMap = new HashMap<Integer, double[]>();
        if (cameraInputs.tagArray.length == 0) { // if packet is empty
            return;
        }

        int packetId = (int) cameraInputs.tagArray[0];
        int cameraId = (int) cameraInputs.tagArray[1];
        if (packetId == cameraLastPacketIds[cameraId]) {
            timesLastCameraPacketWasStale[cameraId]++;
            System.out.println(timesLastCameraPacketWasStale[cameraId]); // !
        } else {
            timesLastCameraPacketWasStale[cameraId] = 0;
        }
        if(timesLastCameraPacketWasStale[cameraId] > 10){
            return;
        }
        cameraLastPacketIds[cameraId] = packetId;
        visionEstimate = updateVision();
        combinedEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRawYaw(),
            drive.getModulePositions()
        );
        combinedEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poses/visionPoses/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public Pose2d updateVision() {
        updateCameraMapWithPacket();
        // combine all camera maps // ! maybe log the camera poses
        for (int i = 0; i < numCameras; i++) { // iterate through all camera maps
            HashMap<Integer, double[]> cameraMap = cameraMaps.get(i);
            for (int key : cameraMap.keySet()) { // iterate through tags in the camera map
                // log the camera map
                Logger.recordOutput("outputs/poseEstimator/maps/camera" + Integer.toString(i) +"Map/" + Integer.toString(key), cameraMap.get(key));

                // increment the tick values
                cameraMap.put(key, new double[]{
                    cameraMap.get(key)[0],
                    cameraMap.get(key)[1],
                    cameraMap.get(key)[2],
                    cameraMap.get(key)[3],
                    cameraMap.get(key)[4] - 1
                });
                
                // add to combinedMap
                if(combinedMap.containsKey(key)){ // average the values
                    combinedMap.put(
                        key,
                        new double[] {
                            (combinedMap.get(key)[0] + cameraMap.get(key)[0]) / 2,
                            (combinedMap.get(key)[1] + cameraMap.get(key)[1]) / 2,
                            (combinedMap.get(key)[2] + cameraMap.get(key)[2]) / 2,
                            (combinedMap.get(key)[3] + cameraMap.get(key)[2]) / 2
                        }
                    );
                    continue;
                }
                combinedMap.put(key, cameraMap.get(key));
            }
        }

        // calculate robot pose
        double accumulatedX = 0;
        double accumulatedY = 0; // numtags = size
        System.out.println("SIZE SIZE SIZE SIZE SIZE, " + combinedMap.size());
        for(int key : combinedMap.keySet()){
            System.out.print(key + ", [");
            for (double d : combinedMap.get(key)) {
                System.out.print(d + ", ");
            }
            System.out.println();
        }
        System.out.println("___________________________");
        for(int key : combinedMap.keySet()){
            Pose2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(key);
            accumulatedX += apriltagLocation.getX() - combinedMap.get(key)[2];
            accumulatedY += apriltagLocation.getY() - combinedMap.get(key)[3];
        }

        double size = combinedMap.size();
        if (size == 0) {
            return new Pose2d();
        }
        return new Pose2d(accumulatedX / size, accumulatedY / size, getRawYaw());
    }

    public void updateCameraMapWithPacket() {
        // get the camera offset
        int cameraId = (int) cameraInputs.tagArray[1];
        Pose2d cameraOffsetFromCenter = new Pose2d();
        switch (cameraId) {
            case 0: 
                cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
                break;
            case 1: 
                cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
                break;
            case 2: 
                cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
                break;
            case 3: 
                cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
                break;
            case 4: 
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
            
            packetMap.put(tagId, new double[]{xFromRobot, yFromRobot, xFromField, yFromField, ticksToKeep});
        }
        
        // update camera map
        HashMap<Integer, double[]> tempCameraMap = new HashMap<Integer, double[]>();
        for (int tagId : packetMap.keySet()) { // add all the new values from packetMap
            tempCameraMap.put(tagId, packetMap.get(tagId));
        }
        for (int key : cameraMaps.get(cameraId).keySet()) { // transferring the old cameraMap values
            if (tempCameraMap.containsKey(key)) { // if that tag was already updated
                continue;
            }
            if (cameraMaps.get(cameraId).get(key)[4] <= 0) {// if the old value is stale
                continue;
            }
            tempCameraMap.put(key, cameraMaps.get(cameraId).get(key));
        }
        cameraMaps.set(cameraId, tempCameraMap);
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