package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;
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

    public DriveWithPosition(
        Drive drive, 
        PoseEstimator poseEstimator,
        int tagId, 
        int level, 
        boolean right
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetPose = VirtualConstants.FIELD_LAYOUT.getTagPose(tagId).get().toPose2d().plus(
            PhysicalConstants.ROBOT_TRANSFORM_MAP.get(level)[right ? 1 : 0]
        );
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = poseEstimator.getPose();
        return Math.abs(robotPose.getX() - targetPose.getX()) < 0.01
            && Math.abs(robotPose.getY() - targetPose.getY()) < 0.01
            && Math.abs(
                MathUtil.inputModulus(robotPose.getRotation().getRotations(), 0, 1)
                - MathUtil.inputModulus(targetPose.getRotation().getRotations(), 0, 1)
            ) < 0.005;
            // && drive.getSpeeds().vxMetersPerSecond < 0.1
            // && drive.getSpeeds().vyMetersPerSecond < 0.1
            // && drive.getSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}