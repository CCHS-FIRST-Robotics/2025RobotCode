package frc.robot.subsystems.PoseEstimator;

import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

// ! motherfucker bad distance detection
public class Camera implements CameraIO{
    NetworkTable tagsTable;
    DoubleArraySubscriber tagSubscriber;

    Drive drive;

    public Camera(Drive drive) {
        tagsTable = NetworkTableInstance.getDefault().getTable("tags"); // ! prolly change the name later
        tagSubscriber = tagsTable.getDoubleArrayTopic("tags").subscribe(new double[] {});

        this.drive = drive;
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        double[] tags = tagSubscriber.get();
        Rotation2d robotYaw = drive.getYaw();

        System.out.println("array thingy: ");
        for(double d : tags){
            System.out.print(d + ", ");
        }
        System.out.println();

        int tagCount = 0;
        double estimatedX = 0;
        double estimatedY = 0;

        // for (int i = 1; i < tags.length; i += 4) {
        //     int tagId = (int) tags[i];

        //     double distance = tags[i+3];
            
        //     double tagAngle = tags[i+1]; // ! oh hey units
        //     double robotAngle = robotYaw.getRadians();
            
        //     double angleToTag = tagAngle - robotAngle;

        //     double dx = Math.sin(angleToTag) * distance; // !oh hey units
        //     double dy = Math.cos(angleToTag) * distance;

        //     Translation3d taglocation = PhysicalConstants.APRILTAG_LOCATIONS.get(tagId);

        //     // estimatedX += taglocation.getX() - dx;
        //     // estimatedY += taglocation.getY() - dy;

        //     tagCount++;
        // }
        // estimatedX /= tagCount;
        // estimatedY /= tagCount;

        // tagsMap.clear(); // ! eh?
        inputs.estimatedPose = new Pose2d(estimatedX, estimatedY, robotYaw);
        // System.out.println("tagCount" + tagCount);
        // System.out.println("estimatedX" + estimatedX);
        // System.out.println("estimatedY" + estimatedY);

    }
}