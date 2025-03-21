package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int targetTagId;
    private final boolean left;
    private double[] poseFromRobotArray; // xDistance, yDistance, angleToTag
    private boolean isFinished = false;

    public DriveWithApriltag(
        Drive drive,
        PoseEstimator poseEstimator,
        int targetTagId,
        boolean left
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetTagId = targetTagId;
        this.left = left;
        poseFromRobotArray = poseEstimator.getArrayFromSpecificTag(targetTagId);
    }

    @Override
    public void execute() {
        poseFromRobotArray = poseEstimator.getArrayFromSpecificTag(targetTagId);

        // finish if tag is not detected
        if (poseFromRobotArray == null) {
            System.out.println("TAGNOTDETECTED");
            isFinished = true;
            return;
        }

        // calculate offsets
        double xOffset = poseFromRobotArray[0]; // meters
        double yOffset = poseFromRobotArray[1] + 0.1651 * (left ? -1 : 1); // meters
        double targetAngle = PhysicalConstants.APRILTAG_LOCATIONS.get(targetTagId).getRotation().getRadians() - Math.PI; // radians
        double oOffset = targetAngle - poseEstimator.getRawYaw().getRadians(); // radians

        System.out.println("oOffset, " + oOffset);

        ChassisSpeeds speeds = new ChassisSpeeds(
            Math.abs(xOffset) > 0.84 ? Math.signum(xOffset) * 0.1 : 0,
            // Math.abs(yOffset) > 0.01 ? Math.signum(yOffset) * 0.1 : 0,
            0,
            Math.abs(oOffset) > 0.01 ? Math.signum(oOffset) * 0.2 : 0
        );

        // finish if offset is low enough
        if (speeds.vxMetersPerSecond == 0
         && speeds.vyMetersPerSecond == 0
         && speeds.omegaRadiansPerSecond == 0
        ) {
            System.out.println("OFFSETLOWENOUGH");
            isFinished = true;
            return; 
        }

        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        System.out.println(isFinished ?  "FINISHED" : null);
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}