package frc.robot.subsystems.poseEstimators;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected;

        public double roll;
        public double pitch;
        public double yaw;
        public double rollVelocity;
        public double pitchVelocity;
        public double yawVelocity;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}