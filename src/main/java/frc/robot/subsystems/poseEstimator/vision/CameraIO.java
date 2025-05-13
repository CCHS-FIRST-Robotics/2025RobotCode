package frc.robot.subsystems.poseEstimator.vision;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {}

    public default void updateInputs(CameraIOInputs inputs) {}
}