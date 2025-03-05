package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;

public class DriveWithPosition extends Command {
    private final Drive drive;
    private final Pose2d pose;
    // note that x is away from the alliance wall and y is to the left
    public DriveWithPosition(
        Drive drive,
        Pose2d pose
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.pose = pose;
    }

    @Override
    public void execute() {
        drive.runPosition(pose);
    }

    @Override
    public boolean isFinished(){
        return false; // ! idk if this is needed
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}