package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int targetTagId;
    private double[] targetTagArray; // xDistance, yDistance, angleToTag

    public DriveWithApriltag(
        Drive drive,
        PoseEstimator poseEstimator,
        int targetTagId
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetTagId = targetTagId;
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);
    }

    @Override
    public void execute() {
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);

        if(targetTagArray == null){
            return;
        }

        drive.runVelocity(new ChassisSpeeds(
            Math.abs(targetTagArray[0]) > 0.1 ? MathUtil.inputModulus(targetTagArray[0],-0.5,0.5) : 0,
            Math.abs(targetTagArray[1]) > 0.1 ? MathUtil.inputModulus(targetTagArray[1],-0.5,0.5) : 0,
            Math.abs(targetTagArray[2]) > 0.1 ? MathUtil.inputModulus(targetTagArray[2],-0.5,0.5) : 0
        ));
    }

    @Override
    public boolean isFinished(){
        return Math.abs(targetTagArray[0]) < 0.05
            && Math.abs(targetTagArray[1]) < 0.05
            && Math.abs(targetTagArray[2]) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}