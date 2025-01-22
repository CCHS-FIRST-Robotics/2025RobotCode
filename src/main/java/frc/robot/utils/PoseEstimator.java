package frc.robot.utils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

// ! pending deletion

public class PoseEstimator extends SwerveDrivePoseEstimator {
    double latestTimestamp = Timer.getFPGATimestamp();
    Pose2d poseEstimate = new Pose2d();

    public PoseEstimator(SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters
    ) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public Pose2d getPoseEstimate() {
        return getEstimatedPosition();
    }

    @Override
    public Pose2d updateWithTime(double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        return super.updateWithTime(timestamp, gyroAngle, modulePositions);
    }
}