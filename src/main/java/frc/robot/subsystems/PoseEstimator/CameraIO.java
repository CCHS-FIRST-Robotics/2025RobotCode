package frc.robot.subsystems.poseEstimator;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs{
        public double[] tagArray = new double[0];
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}