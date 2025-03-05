package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithPosition extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Pose2d targetPose;

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Pose2d targetPose
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished(){
        Pose2d robotPose = poseEstimator.getOdometryPose();
        return Math.abs(robotPose.getX() - targetPose.getX()) < 0.05
            && Math.abs(robotPose.getY() - targetPose.getY()) < 0.05
            && Math.abs(robotPose.getRotation().getRotations() - targetPose.getRotation().getRotations()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}