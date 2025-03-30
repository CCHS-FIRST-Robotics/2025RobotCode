package frc.robot.subsystems.poseEstimator;

import java.util.ArrayList;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    public static class TagWithAmbiguity {
        public AprilTag tag;
        public double ambiguity;

        public TagWithAmbiguity(AprilTag tag, double ambiguity) {
            this.tag = tag;
            this.ambiguity = ambiguity;
        }

        public void toLog(LogTable table, String prefix) {
            table.put(prefix + "ID" + tag.ID +"/X", tag.pose.getX());
            table.put(prefix + "ID" + tag.ID +"/Y", tag.pose.getY());
            table.put(prefix + "ID" + tag.ID +"/Yaw", Math.toDegrees(tag.pose.getRotation().getAngle()));
            table.put(prefix + "ID" + tag.ID +"/Ambiguity", ambiguity);
        }

    }

    public static class VisionIOInputs implements LoggableInputs {
        public boolean connected;

        public String Name;

        public double timestamp;

        public Pose2d VisionPose2d;

        public Pose3d VisionPose3d;

        public int NumTags;

        public ArrayList<TagWithAmbiguity> Tags = new ArrayList<TagWithAmbiguity>();

        @Override
        public void toLog(LogTable table) {
            String cameraPrefix = "Vision/" + Name + "/";

            table.put(cameraPrefix + "connected", connected);
            table.put(cameraPrefix + "timestamp", timestamp);
            
            table.put(cameraPrefix + "VisionPose2d", VisionPose2d);
            table.put(cameraPrefix + "VisionPose3d", VisionPose3d);
            

            table.put(cameraPrefix + "NumTags", NumTags);

           
            for (int i = 0; i < Tags.size(); i++) {
                Tags.get(i).toLog(table, cameraPrefix);
            }
            
               
        }

        @Override
        public void fromLog(LogTable table) {
            String cameraPrefix = "Vision/" + Name + "/";
            connected = table.get(cameraPrefix + "connected", connected);
            timestamp = table.get(cameraPrefix + "timestamp", timestamp);
            VisionPose2d = table.get(cameraPrefix + "VisionPose2d", VisionPose2d);
            VisionPose3d = table.get(cameraPrefix + "VisionPose3d", VisionPose3d);
            NumTags = table.get(cameraPrefix + "NumTags", NumTags);

        }
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}