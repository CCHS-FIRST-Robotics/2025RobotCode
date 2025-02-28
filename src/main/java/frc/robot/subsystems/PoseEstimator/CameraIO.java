package frc.robot.subsystems.PoseEstimator;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class GyroIOInputs {

    }

    public default void updateInputs(GyroIOInputs inputs) {}
}