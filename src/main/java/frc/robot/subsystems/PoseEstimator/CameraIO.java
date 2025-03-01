package frc.robot.subsystems.poseEstimator;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs{
        double[] tagArray;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}