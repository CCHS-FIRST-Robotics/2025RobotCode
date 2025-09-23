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
    
    private Pose2d fieldPosition = new Pose2d(); // only used for rotation

    private final Drive drive;

    public Odometry(GyroIO gyroIO, Drive drive){
        this.gyroIO = gyroIO;
        this.drive = drive;

        startAngle = inputs.yaw;
    }
    
    public void periodic() {
        gyroIO.updateInputs(inputs);
        Logger.processInputs("poseEstimator/gyro", inputs);
        
        Logger.recordOutput("startAngle", startAngle);

        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(drive.getModuleDeltas()));
    }

    public void resetStartAngle(){
        startAngle = inputs.yaw;
    }

    public Rotation2d getYaw(){
        return getRawYaw();
    }

    @SuppressWarnings("unused")
    private Rotation2d getRelativeYaw(){
        return inputs.connected ? new Rotation2d(
            MathUtil.angleModulus(Rotations.of(startAngle).in(Radians))
            - MathUtil.angleModulus(Rotations.of(inputs.yaw).in(Radians))
        ) : fieldPosition.getRotation();
    }

    // @SuppressWarnings("unused")
    private Rotation2d getRawYaw() {
        return inputs.connected ? new Rotation2d(Rotations.of(inputs.yaw).in(Radians)) : fieldPosition.getRotation();
    }
}