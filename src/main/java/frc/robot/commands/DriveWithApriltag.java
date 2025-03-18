package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int targetTagId;
    private final boolean left;
    private double[] targetTagOffsetArray; // xDistance, yDistance, angleToTag
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
        targetTagOffsetArray = poseEstimator.getOffsetFromSpecificTag(targetTagId);
    }

    @Override
    public void execute() {
        targetTagOffsetArray = poseEstimator.getOffsetFromSpecificTag(targetTagId);

        // finish if tag is not detected
        if (targetTagOffsetArray == null) {
            isFinished = true;
            return; 
        }

        // calculate offsets
        double xOffset = targetTagOffsetArray[0]; // meters
        double yOffset = targetTagOffsetArray[1] + 0.1651 * (left ? -1 : 1); // meters
        double oOffset = targetTagOffsetArray[2]; // radians

        ChassisSpeeds speeds = new ChassisSpeeds(
            Math.abs(xOffset) > 0.1 ? 0.1 : 0,
            Math.abs(yOffset) > 0.1 ? 0.1 : 0,
            Math.abs(oOffset) > 0.1 ? 0.1 : 0
        );

        // finish if offset is low enough
        if (speeds.vxMetersPerSecond == 0
         && speeds.vyMetersPerSecond == 0
         && speeds.omegaRadiansPerSecond == 0
        ) {
            isFinished = true;
            return; 
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