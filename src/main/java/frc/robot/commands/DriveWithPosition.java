package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
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

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Transform2d targetTransform
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetPose = poseEstimator.getPose().plus(targetTransform);
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = poseEstimator.getPose();
        return Math.abs(robotPose.getX() - targetPose.getX()) < 0.02
            && Math.abs(robotPose.getY() - targetPose.getY()) < 0.02
            && Math.abs(
                MathUtil.inputModulus(robotPose.getRotation().getRotations(), 0, 1)
                - MathUtil.inputModulus(targetPose.getRotation().getRotations(), 0, 1)
            ) < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        System.out.println("FINISH TRUST FINISH");
    }
}