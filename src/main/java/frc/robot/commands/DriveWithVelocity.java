package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.constants.*;

public class DriveWithVelocity extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Supplier<Double> linearXSpeedSupplier;
    private final Supplier<Double> linearYSpeedSupplier;
    private final Supplier<Double> angularVelocitySupplier;
    
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();

    // note that x is away from the alliance wall and y is to the left
    public DriveWithVelocity(
        Drive drive,
        PoseEstimator poseEstimator,
        Supplier<Double> linearXSpeedSupplier,
        Supplier<Double> linearYSpeedSupplier,
        Supplier<Double> angularVelocitySupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.linearXSpeedSupplier = linearXSpeedSupplier;
        this.linearYSpeedSupplier = linearYSpeedSupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
    }

    @Override
    public void execute() {
        // get linearVelocity
        double linearXSpeed = linearXSpeedSupplier.get() + 2e-6; // + 2e-6 to prevent from constant "x and y components of rotation2d are 0" error
        double linearYSpeed = linearYSpeedSupplier.get() + 2e-6;
        double linearSpeed = applyPreferences(
            Math.hypot(linearXSpeed, linearYSpeed), 
            VirtualConstants.JOYSTICK_DEADZONE, 
            VirtualConstants.LINEAR_SPEED_EXPONENT
        );
        Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed);
        Translation2d linearVelocity = new Translation2d(
            linearSpeed,
            linearDirection
        );

        // get angularVelocity
        double angularVelocity = applyPreferences(
            angularVelocitySupplier.get(), 
            VirtualConstants.JOYSTICK_DEADZONE, 
            VirtualConstants.ANGULAR_SPEED_EXPONENT
        );

        // denormalize speeds and FOC
        ChassisSpeeds speedsInput = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * PhysicalConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond),
            -linearVelocity.getY() * PhysicalConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond), // chassisspeeds is flipped
            -angularVelocity * PhysicalConstants.MAX_ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond), // chassisspeeds is flipped
            poseEstimator.getYawWithAllianceRotation()
        );

        // clamp everything between max and min possible accels
        speedsInput = new ChassisSpeeds(
            clampVelocity(
                speedsInput.vxMetersPerSecond, 
                prevSpeeds.vxMetersPerSecond, 
                PhysicalConstants.MAX_ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * VirtualConstants.PERIOD
            ),
            clampVelocity(
                speedsInput.vyMetersPerSecond, 
                prevSpeeds.vyMetersPerSecond, 
                PhysicalConstants.MAX_ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * VirtualConstants.PERIOD
            ),
            clampVelocity(
                speedsInput.omegaRadiansPerSecond, 
                prevSpeeds.omegaRadiansPerSecond, 
                PhysicalConstants.MAX_ALLOWED_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)) * VirtualConstants.PERIOD
            )
        );

        Logger.recordOutput("outputs/drive/speedsInput", speedsInput);
        prevSpeeds = speedsInput;
        
        drive.runVelocity(speedsInput);
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