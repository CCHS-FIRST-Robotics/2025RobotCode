package frc.robot.subsystems.poseEstimator.odometry;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.GyroIOInputsAutoLogged;

public class Odometry {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    
    private Pose2d fieldPosition = new Pose2d();

    private final Drive drive;

    public Odometry(GyroIO gyroIO, Drive drive){
        this.gyroIO = gyroIO;
        this.drive = drive;
    }
    
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("poseEstimator/gyro", gyroInputs);

        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
    }

    public Rotation2d getRawYaw() {
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }

    public void resetFieldPosition(Pose2d pose){
        fieldPosition = pose;
    }
}
