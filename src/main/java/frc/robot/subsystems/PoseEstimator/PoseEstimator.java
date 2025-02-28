package frc.robot.subsystems.PoseEstimator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.constants.PhysicalConstants;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Camera camera;

    public PoseEstimator(Drive drive){
        poseEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d(1, 1, new Rotation2d())
        );
        
        camera = new Camera(drive);
    }

    @Override
    public void periodic(){
        Logger.recordOutput("outputs/pose/odometryPoseEstimate", poseEstimator.getEstimatedPosition());
    }

    public void resetPosition(Pose2d pose, SwerveModulePosition[] modulePositions){
        poseEstimator.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public void updateWithOdometry(double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){
        poseEstimator.updateWithTime(
            timestamp,
            gyroAngle,
            modulePositions
        );
    }

    public void updateWithVision(){
        // kalman filters or smth idk
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }
}