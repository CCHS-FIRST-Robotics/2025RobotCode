package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class DriveWithApriltag extends Command {
    private enum DriveMode{
        ROTATE,
        Y,
        X
    }
    private DriveMode driveMode = DriveMode.ROTATE;

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int targetTagId;
    private final Translation2d drivePosition;
    private final boolean left;
    private double[] offsetArray; // xDistance, yDistance, angleToTag
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
        this.poseEstimator = poseEstimator;
        this.targetTagId = targetTagId;
        this.drivePosition = drivePosition;
        this.left = left;
        offsetArray = poseEstimator.getArrayFromSpecificTag(targetTagId);
        for(int i = 0; i<5; i++){
            System.out.println("NEW THING, " + isFinished);
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < 5; i++){
            System.out.println("RUNNING");
        }
        offsetArray = poseEstimator.getArrayFromSpecificTag(targetTagId);

        // finish if tag is not detected
        if (offsetArray == null) {
            for (int i = 0; i < 5; i++){
                System.out.println("TAGNOTDETECTED");
            }
            isFinished = true;
            return;
        }

        // calculate error
        double xError = offsetArray[0] - drivePosition.getX(); // meters
        double yError = offsetArray[1] - drivePosition.getY() - 0.14 * (left ? -1 : 1); // meters
        //double yError = offsetArray[1] - drivePosition.getY(); // meters //!  for only middle tracking
        double oOffset = PhysicalConstants.APRILTAG_LOCATIONS.get(targetTagId).getRotation().getRadians() - Math.PI; // radians
        double oError = oOffset - MathUtil.angleModulus(poseEstimator.getRawYaw().getRadians()); // radians
        //double oError = Math.toRadians(25) * (left ? -1 : 1); //! middle tracking (25 degress is wrong)
        // double xKp = 1; //! these  p's are for pid version
        // double yKp = 1;
        // double oKp = 1;
        // ! but what if gyro is pi and target is -pi

        Logger.recordOutput("oOffset", oOffset);
        Logger.recordOutput("yOffset", offsetArray[1]);
        Logger.recordOutput("xOffset", offsetArray[0]);

        Logger.recordOutput("oError", oError);
        Logger.recordOutput("yError", yError);
        Logger.recordOutput("xError", xError);

        ChassisSpeeds speeds = new ChassisSpeeds();
        switch (driveMode) {
            case ROTATE: 
                speeds = new ChassisSpeeds(
                    0,
                    0,
                    Math.abs(oError) > 0.001 ? Math.signum(oError) * 0.3 : 0
                );
                if (speeds.omegaRadiansPerSecond == 0) {
                    driveMode = DriveMode.Y;
                }
                break;
            case Y: 
                speeds = new ChassisSpeeds(
                    0,
                    Math.abs(yError) > 0.001 ? Math.signum(yError) * 0.1 : 0,
                    0
                );
                if (speeds.vyMetersPerSecond == 0) {
                    driveMode = DriveMode.X;
                    return;
                }
                break;
            case X: 
                speeds = new ChassisSpeeds(
                    Math.abs(xError) > 0.001 ? Math.signum(xError) * 0.1 : 0,
                    0,
                    0
                );
                if (speeds.vxMetersPerSecond == 0) {
                    isFinished = true;
                    return; 
                }
                break;
        }


        // ! pid version
        // ChassisSpeeds speeds = new ChassisSpeeds();
        // switch (driveMode) {
        //     case ROTATE: 
        //         speeds = new ChassisSpeeds(
        //             0,
        //             0,
        //             Math.abs(oError) > 0.001 ? oError * oKp : 0
        //         );
        //         if (speeds.omegaRadiansPerSecond == 0) {
        //             driveMode = DriveMode.Y;
        //         }
        //         break;
        //     case Y: 
        //         speeds = new ChassisSpeeds(
        //             0,
        //             Math.abs(yError) > 0.001 ? yError * yKp : 0,
        //             0
        //         );
        //         if (speeds.vyMetersPerSecond == 0) {
        //             driveMode = DriveMode.X;
        //             return;
        //         }
        //         break;
        //     case X: 
        //         speeds = new ChassisSpeeds(
        //             Math.abs(xError) > 0.001 ? xError * xKp: 0,
        //             0,
        //             0
        //         );
        //         if (speeds.vxMetersPerSecond == 0) {
        //             isFinished = true;
        //             return; 
        //         }
        //         break;
        // }







            //! home to middle then turn # radians (need to make arm raised a bit to use his)

        // ChassisSpeeds speeds = new ChassisSpeeds();
        // switch (driveMode) {
        //     case ROTATE: 
        //         speeds = new ChassisSpeeds(
        //             0,
        //             0,
        //             Math.abs(oError) > 0.001 ? Math.signum(oError) * 0.3 : 0
        //         );
        //         if (speeds.omegaRadiansPerSecond == 0) {
        //             driveMode = DriveMode.Y;
        //         }
        //         break;
        //     case Y: 
        //         speeds = new ChassisSpeeds(
        //             0,
        //             Math.abs(yError) > 0.001 ? Math.signum(yError) * 0.1 : 0,
        //             0
        //         );
        //         if (speeds.vyMetersPerSecond == 0) {
        //             driveMode = DriveMode.X;
        //             return;
        //         }
        //         break;
        //     case X: 
        //         speeds = new ChassisSpeeds(
        //             Math.abs(xError) > 0.001 ? Math.signum(xError) * 0.1 : 0,
        //             0,
        //             0
        //         );
        //         if (speeds.vxMetersPerSecond == 0) {
        //             isFinished = true;
        //             return; 
        //         }
        //         break;
        // }




        


        speeds = new ChassisSpeeds(
            Math.abs(xError) > 0.005 ? Math.signum(xError) * 0.2 : 0,
            Math.abs(yError) > 0.005 ? Math.signum(yError) * 0.2 : 0,
            Math.abs(oError) > 0.005 ? Math.signum(oError) * 0.4 : 0
        );

        speeds = new ChassisSpeeds ( // PID
            Math.abs(xError) > 0.005 ? xError * 2 : 0,
            Math.abs(yError) > 0.005 ? yError * 2 : 0,
            Math.abs(oError) > 0.005 ? oError * 0.5 : 0
        );

        if (speeds.vxMetersPerSecond == 0
         && speeds.vyMetersPerSecond == 0
         && speeds.omegaRadiansPerSecond == 0
        ) {
            for (int i = 0; i < 5; i++){
                System.out.println("SPEEDIS0");
            }
            isFinished = true;
            return; 
        }

        for (int i = 0; i < 5; i++){
            System.out.println("RUNNINGVELOCITY");
        }
        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 5; i++){
            System.out.println("FINISHED");
        }
        isFinished = false; // ! sigh I wish this didn't have to exist
        drive.stop();
    }
}