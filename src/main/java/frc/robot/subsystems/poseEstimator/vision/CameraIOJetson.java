package frc.robot.subsystems.poseEstimator.vision;

// import edu.wpi.first.networktables.*;

public class CameraIOJetson implements CameraIO {
    // private final NetworkTable tagsTable;
    // private final DoubleArraySubscriber tagSubscriber;

    // public CameraIOJetson() {
    //     tagsTable = NetworkTableInstance.getDefault().getTable("tags");
    //     tagSubscriber = tagsTable.getDoubleArrayTopic("tags").subscribe(new double[] {});
    // }

    // @Override
    // public void updateInputs(CameraIOInputs inputs) {
    //     // inputs.tagArray = tagSubscriber.get();
    // }

    // ! fuck you, figure this out later
            // cameraLastPacketIds = new int[numCameras];
        // timesLastCameraPacketWasStale = new int[numCameras];
        // cameraMaps = new ArrayList<HashMap<Integer, double[]>>();
        // for(int i = 0; i < numCameras; i++){
        //     cameraMaps.add(new HashMap<Integer, double[]>());
        // }
    // private final int ticksToKeep = 10;
    // private final int numCameras = 5;
    // private final int[] cameraLastPacketIds;
    // private final int[] timesLastCameraPacketWasStale;
    // private final ArrayList<HashMap<Integer, double[]>> cameraMaps;
    // private HashMap<Integer, double[]> combinedMap = new HashMap<Integer, double[]>();
    // combinedMap = new HashMap<Integer, double[]>();
    // for(int i = 1; i <= 22; i++) {
    //     Logger.recordOutput("outputs/poseEstimator/maps/combinedMap/" + Integer.toString(i), combinedMap.get(i));
    // }
    // if (cameraInputs.tagArray.length == 0) { // if packet is empty
    //     return;
    // }
    // int packetId = (int) cameraInputs.tagArray[0];
    // int cameraId = (int) cameraInputs.tagArray[1];
    // if (packetId == cameraLastPacketIds[cameraId]) {
    //     timesLastCameraPacketWasStale[cameraId]++;
    //     Logger.recordOutput("stale", timesLastCameraPacketWasStale[cameraId]);
    // } else {
    //     timesLastCameraPacketWasStale[cameraId] = 0;
    // }
    // if (timesLastCameraPacketWasStale[cameraId] > 10) {
    //     return;
    // }
    // cameraLastPacketIds[cameraId] = packetId;


    // public Pose2d updateVision() {
    //     updateCameraMapWithPacket();
    //     // combine all camera maps // ! maybe log the camera poses
    //     for (int i = 0; i < numCameras; i++) { // iterate through all camera maps
    //         HashMap<Integer, double[]> cameraMap = cameraMaps.get(i);
    //         for (int key : cameraMap.keySet()) { // iterate through tags in the camera map
    //             // increment the tick values
    //             cameraMap.put(key, new double[]{
    //                 cameraMap.get(key)[0],
    //                 cameraMap.get(key)[1],
    //                 cameraMap.get(key)[2],
    //                 cameraMap.get(key)[3],
    //                 cameraMap.get(key)[4] - 1
    //             });
                
    //             // add to combinedMap
    //             if (combinedMap.containsKey(key)) { // average the values
    //                 combinedMap.put(
    //                     key,
    //                     new double[] {
    //                         (combinedMap.get(key)[0] + cameraMap.get(key)[0]) / 2,
    //                         (combinedMap.get(key)[1] + cameraMap.get(key)[1]) / 2,
    //                         (combinedMap.get(key)[2] + cameraMap.get(key)[2]) / 2,
    //                         (combinedMap.get(key)[3] + cameraMap.get(key)[2]) / 2
    //                     }
    //                 );
    //                 continue;
    //             }
    //             combinedMap.put(key, cameraMap.get(key));
    //         }
    //     }

    //     for(int i = 1; i <= 22; i++) {
    //         Logger.recordOutput("outputs/poseEstimator/maps/combinedMap/" + Integer.toString(i), combinedMap.get(i));
    //     }

    //     // calculate robot pose
    //     double accumulatedX = 0;
    //     double accumulatedY = 0; // numtags = size
    //     // for(int key : combinedMap.keySet()){
    //     //     System.out.print(key + ", [");
    //     //     for (double d : combinedMap.get(key)) {
    //     //         System.out.print(d + ", ");
    //     //     }
    //     //     System.out.println();
    //     // }
    //     // System.out.println("___________________________");
    //     for(int key : combinedMap.keySet()){
    //         Pose2d apriltagLocation = PhysicalConstants.APRILTAG_LOCATIONS.get(key);
    //         accumulatedX += apriltagLocation.getX() - combinedMap.get(key)[2];
    //         accumulatedY += apriltagLocation.getY() - combinedMap.get(key)[3];
    //     }

    //     double size = combinedMap.size();
    //     if (size == 0) {
    //         return new Pose2d();
    //     }
    //     return new Pose2d(accumulatedX / size, accumulatedY / size, getRawYaw());
    // }

    // public void updateCameraMapWithPacket() {
    //     // get the camera offset
    //     int cameraId = (int) cameraInputs.tagArray[1];
    //     Pose2d cameraOffsetFromCenter = new Pose2d();
    //     switch (cameraId) {
    //         case 0, 1, 2, 3: 
    //             cameraOffsetFromCenter = PhysicalConstants.JETSON_OFFSET;
    //             break;
    //         case 4: 
    //             cameraOffsetFromCenter = PhysicalConstants.USB_CAMERA_1_OFFSET;
    //             break;
    //     }
        
    //     // get the packetMap
    //     HashMap<Integer, double[]> packetMap = new HashMap<Integer, double[]>();     
    //     for (int i = 2; i < cameraInputs.tagArray.length; i += 4) {
    //         // get networktables values
    //         int tagId = (int) cameraInputs.tagArray[i];
    //         double tagYaw = -cameraInputs.tagArray[i+1]; // radians; sleder's code does clockwise positive
    //         double tagPitch = cameraInputs.tagArray[i+2]; // radians
    //         double tagDistance = cameraInputs.tagArray[i+3]; // meters
    //         double robotYaw = getRawYaw().getRadians();

    //         // get distance from camera
    //         double pitchToTag = tagPitch;
    //         double hypotFromCamera = Math.cos(pitchToTag) * tagDistance;

    //         // get values from camera
    //         double yawFromCamera = tagYaw;
    //         double xFromCamera = Math.cos(yawFromCamera) * hypotFromCamera;
    //         double yFromCamera = Math.sin(yawFromCamera) * hypotFromCamera;

    //         // get values from robot center
    //         double xFromRobot = xFromCamera + cameraOffsetFromCenter.getX();
    //         double yFromRobot = yFromCamera + cameraOffsetFromCenter.getY();
    //         double hypotFromRobot = Math.hypot(xFromRobot, yFromRobot);
    //         double yawFromRobot = Math.acos(xFromRobot / hypotFromRobot);
            
    //         // get values from robot center relative to the field 
    //         double yawFromField = robotYaw + yawFromRobot;
    //         double xFromField = Math.cos(yawFromField) * hypotFromRobot;
    //         double yFromField = Math.sin(yawFromField) * hypotFromRobot;
            
    //         packetMap.put(tagId, new double[]{xFromRobot, yFromRobot, xFromField, yFromField, ticksToKeep});
    //     }
    
    //     // update camera map
    //     HashMap<Integer, double[]> cameraMap = new HashMap<Integer, double[]>();
    //     for (int tagId : packetMap.keySet()) { // add all the new values from packetMap
    //         cameraMap.put(tagId, packetMap.get(tagId));
    //     }
    //     for (int key : cameraMaps.get(cameraId).keySet()) { // transferring the old cameraMap values
    //         if (cameraMap.containsKey(key)) { // if that tag was already updated
    //             continue;
    //         }
    //         if (cameraMaps.get(cameraId).get(key)[4] <= 0) {// if the old value is stale
    //             continue;
    //         }
    //         cameraMap.put(key, cameraMaps.get(cameraId).get(key));
    //     }
    //     cameraMaps.set(cameraId, cameraMap);
    // }
}