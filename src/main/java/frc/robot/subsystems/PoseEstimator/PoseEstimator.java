package frc.robot.subsystems.PoseEstimator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.CameraIOInputsAutoLogged;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Camera camera;
    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    public PoseEstimator(Drive drive){
        poseEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(), 
            new Pose2d()
        );
        
        camera = new Camera(drive);
    }

    @Override
    public void periodic(){
        camera.updateInputs(cameraInputs);
        Logger.processInputs("camera", cameraInputs);

        Logger.recordOutput("outputs/drive/robotPose", getPose());
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

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}