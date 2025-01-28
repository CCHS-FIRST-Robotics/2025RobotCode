package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.math.MathUtil;
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
        Tag tag = camera.getTag(followID);
        ChassisSpeeds speeds = new ChassisSpeeds();

        if (tag == null) {
            state = FollowApriltagStates.IDLE;
            return;
        }

        if(tag.getDistance() < Constants.STOP_TAG_DISTANCE){
            speeds = new ChassisSpeeds(0, 0, 0);
            state = FollowApriltagStates.IDLE;
            return;
        }

        if(tag.getAngle() > 0){
            speeds = new ChassisSpeeds(0, 0, -0.5);
        }
        else{
            speeds = new ChassisSpeeds(0, 0, 0.5);
        }
        
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyPreferences(double input, double deadzone, double exponent) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    public double clampVelocity(double velocity, double prevVelocity, double maxAcceleration){
        return MathUtil.clamp(velocity, prevVelocity - maxAcceleration, prevVelocity + maxAcceleration);
    }
}