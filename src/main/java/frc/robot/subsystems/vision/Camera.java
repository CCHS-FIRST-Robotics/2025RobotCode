package frc.robot.subsystems.vision;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Translation3d;

public class Camera {
    NetworkTable tagsTable;
    DoubleArraySubscriber tag_Sub;
    HashMap<Integer, Translation3d> tagsMap;
    Drive drive;
    double robotX;
    double robotY;

    public Camera(Drive drive) {
        this.drive = drive;

        tagsTable = NetworkTableInstance.getDefault().getTable("tag");

        tag_Sub = tagsTable.getDoubleArrayTopic("tags").subscribe(new double[] {});

        tagsMap = new HashMap<Integer, Translation3d>();

        tagsMap.put(1, new Translation3d(16.697, 0.655, 1.486));
        tagsMap.put(2, new Translation3d(16.697, 7.396, 1.486));
        tagsMap.put(3, new Translation3d(11.561, 8.056, 1.302));
        tagsMap.put(4, new Translation3d(9.276, 6.138, 1.868));
        tagsMap.put(5, new Translation3d(9.276, 1.915, 1.868));
        tagsMap.put(6, new Translation3d(13.474, 3.306, 0.308));
        tagsMap.put(7, new Translation3d(13.890, 4.026, 0.308));
        tagsMap.put(8, new Translation3d(13.474, 4.745, 0.308));
        tagsMap.put(9, new Translation3d(12.643, 4.745, 0.308));
        tagsMap.put(10, new Translation3d(12.227, 4.026, 0.308));
        tagsMap.put(11, new Translation3d(12.643, 3.306, 0.308));
        tagsMap.put(12, new Translation3d(0.851, 0.655, 1.486));
        tagsMap.put(13, new Translation3d(0.851, 7.396, 1.486));
        tagsMap.put(14, new Translation3d(8.272, 6.138, 1.868));
        tagsMap.put(15, new Translation3d(8.272, 1.915, 1.868));
        tagsMap.put(16, new Translation3d(5.988, -0.004, 1.302));
        tagsMap.put(17, new Translation3d(4.074, 3.306, 0.308));
        tagsMap.put(18, new Translation3d(3.658, 4.026, 0.308));
        tagsMap.put(19, new Translation3d(4.074, 4.745, 0.308));
        tagsMap.put(20, new Translation3d(4.905, 4.745, 0.308));
        tagsMap.put(21, new Translation3d(5.321, 4.026, 0.308));
        tagsMap.put(22, new Translation3d(4.905, 3.306, 0.308));

    }
    
    public Translation3d getTagLocation(int ID) {
        if (tagsMap.containsKey(ID)) {
            return tagsMap.get(ID);
        }
        return null;
    }

    public void updateInputs() {
        double[] tags = tag_Sub.get();
        Rotation2d robotYaw =  drive.getYaw();

        System.out.println();
        int tagCount = 0;
        robotX = 0;
        robotY = 0;

        for (int i = 1; i < tags.length; i+=3) {
            int tagid = (int)tags[i];
            double tagAngle = tags[i+1];
            double distance = tags[i+2];
            double angle2tag = tagAngle - robotYaw.getRadians();

            double dx = Math.sin(angle2tag) * distance;
            double dy = Math.cos(angle2tag) * distance;

            Translation3d taglocation = getTagLocation(tagid);

            robotX += taglocation.getX() - dx;
            robotY += taglocation.getY() - dy;

            tagCount++;
        
        }
        robotX /= tagCount;
        robotY /= tagCount;

        tagsMap.clear();

    }
}
