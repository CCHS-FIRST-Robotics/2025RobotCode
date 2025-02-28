package frc.robot.subsystems.PoseEstimator;

import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;
public class Camera { // ! make this not act like a subsystem
    NetworkTable tagsTable;
    DoubleArraySubscriber tagSubscriber;

    Drive drive;

    public Camera(Drive drive) {
        tagsTable = NetworkTableInstance.getDefault().getTable("tags");
        tagSubscriber = tagsTable.getDoubleArrayTopic("tags").subscribe(new double[] {});

        this.drive = drive;
    }

    public Pose2d getEstimation() {
        double[] tags = tagSubscriber.get();
        Rotation2d robotYaw = drive.getYaw();

        // System.out.println("array thingy: ");
        // for(double d : tags){
        //     System.out.print(d + ", ");
        // }
        // System.out.println();

        int tagCount = 0;
        double estimatedX = 0;
        double estimatedY = 0;
        for (int i = 1; i < tags.length; i += 4) { // tags[0] is the 
            int id = (int) tags[i];
            double distance = tags[i+3]; // meters
            double tagAngle = tags[i+1]; // radians
            double robotAngle = robotYaw.getRadians();
            double angleToTag = tagAngle - robotAngle;
            double dx = Math.sin(angleToTag) * distance;
            double dy = Math.cos(angleToTag) * distance;

            Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(id);
            estimatedX += taglocation.getX() - dx;
            estimatedY += taglocation.getY() - dy;

            tagCount++;
        }
        estimatedX /= tagCount;
        estimatedY /= tagCount;

        return new Pose2d(estimatedX, estimatedY, robotYaw);
    }
}