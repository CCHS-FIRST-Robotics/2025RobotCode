package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public double[] tagArray = new double[0];
        public Pose2d poseEstimate = new Pose2d();
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}