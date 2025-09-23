package frc.robot.subsystems.poseEstimator.odometry;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.Drive;

public class Odometry {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    
    private Pose2d fieldPosition = new Pose2d(); // only used for rotation

    private final Drive drive;

    public Odometry(GyroIO gyroIO, Drive drive){
        this.gyroIO = gyroIO;
        this.drive = drive;
    }
    
    public void periodic() {
        gyroIO.updateInputs(inputs);
        Logger.processInputs("poseEstimator/gyro", inputs);

        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
    }

    public Rotation2d getRawYaw() {
        return inputs.connected ? new Rotation2d(Rotations.of(inputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }
}