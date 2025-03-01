package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.networktables.*;

public class CameraIOReal implements CameraIO{
    NetworkTable tagsTable;
    DoubleArraySubscriber tagSubscriber;

    public CameraIOReal() {
        tagsTable = NetworkTableInstance.getDefault().getTable("tags");
        tagSubscriber = tagsTable.getDoubleArrayTopic("tags").subscribe(new double[] {});
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.tagArray = tagSubscriber.get();
    }
}