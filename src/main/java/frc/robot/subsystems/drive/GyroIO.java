package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        public double roll;
        public double pitch;
        public double yaw;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}