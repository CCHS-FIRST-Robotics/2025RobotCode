package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class DriveWithJoysticks extends Command{
    Drive drive;
    Supplier<Double> leftYSupplier;
    Supplier<Double> rightXSupplier;

    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> leftYSupplier,
        Supplier<Double> rightXSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
    }

    @Override
    public void execute() {
        double leftY = leftYSupplier.get();
        double rightX = rightXSupplier.get();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            applyPreferences(leftY) * Constants.MAX_SPEED.in(MetersPerSecond),
            0, 
            applyPreferences(rightX) * 5 * -1 // chassisspeeds considers rotating clockwise as positive
        );

        drive.setVelocity(chassisSpeeds);
    }

    public double applyPreferences(double input){
        if(Math.abs(input) < Constants.JOYSTICK_DEADZONE){
            return 0; 
        }
        return Math.signum(input) * Math.pow(Math.abs(input), Constants.JOYSTICK_EXPONENT);
    }
}