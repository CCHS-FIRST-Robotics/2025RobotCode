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
    double Xerr;
    double distance;


    public DriveWithApriltag(
        Drive drive,
        PoseEstimator poseEstimator,
        int targetTagId,
        int distance

      ) {
        addRequirements(drive);
        addRequirements(poseEstimator);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetTagId = targetTagId;
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);
        this.Xerr = 0.0;
        this.distance = distance;
    }

    @Override
    public void execute() {
        targetTagArray = poseEstimator.getSpecificTag(targetTagId);
        Xerr = targetTagArray[1] - this.distance;

        if(targetTagArray == null){
            return;
        }

        // ! just have one drive.runVelocity statement that has a function in each variable


            drive.runVelocity(new ChassisSpeeds(
                Math.abs(targetTagArray[0]) > 0.1 ? MathUtil.inputModulus(targetTagArray[0],-1,1) : 0,
                Math.abs(targetTagArray[1]) > 0.1 ? MathUtil.inputModulus(targetTagArray[1],-1,1) : 0,
                Math.abs(targetTagArray[2]) > 0.1 ? MathUtil.inputModulus(targetTagArray[2],-1,1) : 0
            ));

        if(targetTagArray[1] < -Xerr){
            drive.runVelocity(new ChassisSpeeds(
                0, -Xerr, 0
            ));
            return;
        }
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