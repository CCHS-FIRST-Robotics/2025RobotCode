package frc.robot.subsystems.vision;

import java.util.HashMap;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    NetworkTable tagsTable;

    IntegerArraySubscriber tagIdsSub;
    DoubleArraySubscriber tag_anglesSub;
    DoubleArraySubscriber tag_distancesSub;
    IntegerSubscriber packet_id;
    HashMap<Long, Tag> tags;

    public Camera() {
        tagsTable = NetworkTableInstance.getDefault().getTable("tag");

        tagIdsSub = tagsTable.getIntegerArrayTopic("tagID").subscribe(new long[] {});
        tag_anglesSub = tagsTable.getDoubleArrayTopic("tag_angle").subscribe(new double[] {});
        tag_distancesSub = tagsTable.getDoubleArrayTopic("tag_distance").subscribe(new double[] {});
        packet_id = tagsTable.getIntegerTopic("packet_id").subscribe(0);
        tags = new HashMap<Long, Tag>();

    }
    public Tag getTag(long ID) {
        if (tags.containsKey(ID)) {
            return tags.get(ID);
        }
        return null;
    }

    public void updateInputs() {
        long[] tagids = tagIdsSub.get();
        double[] tag_distances = tag_distancesSub.get();
        double[] tag_angles = tag_anglesSub.get();

        System.out.println(packet_id);

        for (int i = 0; i < tagids.length; i++) {
            System.out.println(tagids[i]);
            System.out.println(tag_distances[i]);
            System.out.println(tag_angles[i]);
        }

        if (tagids.length != tag_angles.length && 
            tagids.length != tag_distances.length) {
            return;
        }
        tags.clear();
        for (int i = 0; i < tagids.length; i++) {
            long id = tagids[i];
            tags.put(id, new Tag(id, tag_distances[i], tag_angles[i]));
        }
    }
}
