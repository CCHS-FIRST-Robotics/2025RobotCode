package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

import frc.robot.subsystems.vision.*;

public class FollowApriltag extends Command {
    Drive drive;
    Camera camera;
    int followID = 0;
    FollowApriltagStates state;

    enum FollowApriltagStates {
        IDLE,
        FOLLOWING
    }

    public FollowApriltag(Drive drive) {
        state = FollowApriltagStates.IDLE;
        addRequirements(drive);
        this.drive = drive;
    }

    public FollowApriltagStates getState() {
        return state;
    }

    public void startFollow(int followID) {
        state = FollowApriltagStates.FOLLOWING;
        this.followID = followID;
        execute();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds();

        Translation3d location = camera.getTagLocation(followID);

        if (location == null) {
            state = FollowApriltagStates.IDLE;
            return;
        }

        if(location.getX() < 0){
            speeds = new ChassisSpeeds(0, 0, 0.5);
            state = FollowApriltagStates.IDLE;
            return;
        }

        if(location.getX() > 0){
            speeds = new ChassisSpeeds(0, 0, -0.5);
        }
        else{
            speeds = new ChassisSpeeds(0.5, 0, 0);
        }
        
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    public double clampVelocity(double velocity, double prevVelocity, double maxAcceleration){
        return MathUtil.clamp(velocity, prevVelocity - maxAcceleration, prevVelocity + maxAcceleration);
    }
}