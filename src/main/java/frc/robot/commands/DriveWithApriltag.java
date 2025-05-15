package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private final Drive drive;
    private boolean isFinished = false;

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
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = false;
        drive.stop();
    }
}