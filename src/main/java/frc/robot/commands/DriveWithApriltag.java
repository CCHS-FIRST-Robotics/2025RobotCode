package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.util.Optional;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private final Drive drive;
    private boolean isFinished = false;
    private Pose2d currentPose;
    Optional<Pose3d> where_BE;

    public DriveWithApriltag(
        Drive drive,
        int targetTagId
    ) {
        addRequirements(drive);
        this.drive = drive;
    
        
        this.where_BE = PhysicalConstants.APRILTAG_LAYOUT.getTagPose(targetTagId);
        if(where_BE.isEmpty()){
            end(true);
        }

       

    }

    @Override
    public void execute() {
        drive.runPosition(where_BE.get().toPose2d().plus(new Transform2d(1,0,new Rotation2d(0))));
        
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