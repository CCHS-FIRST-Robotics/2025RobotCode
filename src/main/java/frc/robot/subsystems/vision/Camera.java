package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tags");

    DoubleSubscriber tagId = tagsTable.getDoubleTopic("tagID").subscribe(-1);
    DoubleSubscriber tag_angle = tagsTable.getDoubleTopic("tag_angle").subscribe(-1);

    public double getTagAngle() {
        return tag_angle.get();
    }

}
