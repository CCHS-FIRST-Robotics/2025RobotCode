package frc.robot.subsystems.poseEstimator.odometry;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.Drive;

public class Odometry {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    private double startAngle;
    
    private Pose2d fieldPosition = new Pose2d();

    private final Drive drive;

    public Odometry(GyroIO gyroIO, Drive drive){
        this.gyroIO = gyroIO;
        this.drive = drive;

        startAngle = inputs.connected ? inputs.yaw : 0;
    }
    
    public void periodic() {
        gyroIO.updateInputs(inputs);
        Logger.processInputs("poseEstimator/gyro", inputs);
        Logger.recordOutput("startAngle", startAngle);

        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
    }

    public void resetGyroStartAngle(Pose2d pose){
        startAngle = inputs.yaw;
    }

    public void resetFieldPosition(Pose2d pose){
        fieldPosition = pose;
    }

    public Rotation2d getYaw(){
        return getRawYaw();
    }

    // @SuppressWarnings("unused")
    private Rotation2d getRawYaw() {
        return inputs.connected ? new Rotation2d(Rotations.of(inputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }

    @SuppressWarnings("unused")
    private Rotation2d getRelativeYaw() {
        return inputs.connected ? new Rotation2d(
            MathUtil.angleModulus(Rotations.of(startAngle - inputs.yaw).in(Radians)) // ! don't know if this is okay
        ) : fieldPosition.getRotation();
    }
}