package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public Pose2d poseEstimate = new Pose2d();
        public Transform2d[] tagOffsets;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}