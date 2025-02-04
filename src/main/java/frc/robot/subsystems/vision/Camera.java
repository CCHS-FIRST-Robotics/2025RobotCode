package frc.robot.subsystems.vision;

import java.util.HashMap;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tag");

    IntegerArraySubscriber tagIdsSub = tagsTable.getIntegerArrayTopic("tagID").subscribe(new long[] {});
    DoubleArraySubscriber tag_anglesSub = tagsTable.getDoubleArrayTopic("tag_angle").subscribe(new double[] {});
    DoubleArraySubscriber tag_distancesSub = tagsTable.getDoubleArrayTopic("tag_distance").subscribe(new double[] {});
    HashMap<Long, Tag> tags = new HashMap<Long, Tag>();

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
