package frc.robot.subsystems.vision;
import frc.robot.utils.AprilTag;
import java.util.ArrayList;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MarkerHome {
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tags");

    DoubleSubscriber closestTagId = tagsTable.getDoubleTopic("closest_tagID").subscribe(-1);
    DoubleSubscriber closestTagXdist = tagsTable.getDoubleTopic("closest_tag_x").subscribe(-1);
    DoubleSubscriber closestTagYdist = tagsTable.getDoubleTopic("closest_tag_y").subscribe(-1);
    DoubleSubscriber closestTagHeading = tagsTable.getDoubleTopic("closest_tag_heading").subscribe(-1);


    DoubleArraySubscriber otherTagIds = tagsTable.getDoubleArrayTopic("other_tagIDs").subscribe(new double[] {});
    DoubleArraySubscriber otherTagX = tagsTable.getDoubleArrayTopic("other_tags_x").subscribe(new double[] {});
    DoubleArraySubscriber otherTagY = tagsTable.getDoubleArrayTopic("other_tags_y").subscribe(new double[] {});
    DoubleArraySubscriber otherTagHeadings = tagsTable.getDoubleArrayTopic("other_tag_headings").subscribe(new double[] {});

    public void updateInputs() {
    double[] tagIds = otherTagIds.get();
    double[] tagXs = otherTagX.get();
    double[] tagYs = otherTagY.get();
    double[] tagHeadings = otherTagHeadings.get();

    ArrayList<AprilTag> tags = new ArrayList<AprilTag>(tagIds.length);

    for (int i = 0; i < tagIds.length; i++) {
        tags.add(new AprilTag((int) tagIds[i], tagXs[i], tagYs[i], tagHeadings[i]));
    }
    
}
}
