package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.Constants;
import frc.robot.HardwareConstants;

import frc.robot.subsystems.vision.*;

public class FollowApriltag extends Command {
    Drive drive;
    Camera camera;

    
    ChassisSpeeds prevSpeeds;

    public FollowApriltag(
        Drive drive
    ) {
        addRequirements(drive);
        this.drive = drive;
    }

    @Override
    public void execute() {
        double angle = camera.getTagAngle();

        ChassisSpeeds speeds = new ChassisSpeeds();
        if(angle > 0){
            speeds = new ChassisSpeeds(0, -0.5, 0);
        }
        else{
            speeds = new ChassisSpeeds(0, 0.5, 0);
        }
        
        drive.runVelocity(speeds);
        prevSpeeds = speeds;
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