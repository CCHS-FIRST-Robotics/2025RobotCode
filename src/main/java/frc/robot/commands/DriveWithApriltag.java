package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private enum DriveMode{
        TRANSLATE,
        ROTATE
    }
    private DriveMode driveMode = DriveMode.ROTATE;

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int targetTagId;
    private final Translation2d drivePosition;
    private final boolean left;
    private double[] offsetArray; // xDistance, yDistance, angleToTag
    private boolean isFinished = false;

    // offset = from apriltag
    // error = from intended position

    public DriveWithApriltag(
        Drive drive,
        PoseEstimator poseEstimator,
        int targetTagId,
        Translation2d drivePosition,
        boolean left
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetTagId = targetTagId;
        this.drivePosition = drivePosition;
        this.left = left;
        offsetArray = poseEstimator.getArrayFromSpecificTag(targetTagId);
    }

    @Override
    public void execute() {
        offsetArray = poseEstimator.getArrayFromSpecificTag(targetTagId);

        // finish if tag is not detected
        if (offsetArray == null) {
            isFinished = true;
            return;
        }

        // calculate error
        double xError = offsetArray[0] - drivePosition.getX(); // meters
        double yError = offsetArray[1] - drivePosition.getY() - 0.1651 * (left ? -1 : 1); // meters
        double targetAngle = PhysicalConstants.APRILTAG_LOCATIONS.get(targetTagId).getRotation().getRadians() - Math.PI; // radians
        double oError = targetAngle - poseEstimator.getRawYaw().getRadians(); // radians

        ChassisSpeeds speeds = new ChassisSpeeds();
        switch(driveMode){
            case ROTATE: 
                speeds = new ChassisSpeeds(
                    0,
                    0,
                    Math.abs(oError) > 0.005 ? Math.signum(oError) * 0.2 : 0
                );
                if (speeds.omegaRadiansPerSecond == 0) {
                    driveMode = DriveMode.TRANSLATE;
                }
                break;
            case TRANSLATE: 
                speeds = new ChassisSpeeds(
                    Math.abs(xError) > 0.005 ? Math.signum(xError) * 0.1 : 0,
                    Math.abs(yError) > 0.005 ? Math.signum(yError) * 0.1 : 0,
                    0
                );
                if (speeds.vxMetersPerSecond == 0
                && speeds.vyMetersPerSecond == 0
                ) {
                    isFinished = true;
                    return; 
                }
                break;
        }

        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}