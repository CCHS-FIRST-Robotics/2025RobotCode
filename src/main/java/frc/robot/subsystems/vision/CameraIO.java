package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs{
        public Pose2d estimatedPose;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}