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
    private double[] targetTagArray; // xDistance, yDistance, angleToTag
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
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);
    }

    @Override
    public void execute() {
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);

        if (targetTagArray == null) {
            isFinished = true;
            return; 
        }

        double xError = targetTagArray[0]; // meters
        double yError = targetTagArray[1] + 0.1651 * (left ? -1 : 1); // meters // ! idk if this logic is correct
        double oError = targetTagArray[2]; // radians

        ChassisSpeeds speeds = new ChassisSpeeds( // ! add the rampdown later
            Math.abs(xError) > 0.1 ? 0.25 : 0,
            Math.abs(yError) > 0.1 ? 0.25 : 0,
            Math.abs(oError) > 0.1 ? 0.10 : 0
        );

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