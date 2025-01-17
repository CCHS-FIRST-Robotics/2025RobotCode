package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

// TODO: do we need any raw acc data? I don't think we have any use for it
// TODO: dual imu??? https://arxiv.org/pdf/2107.02632.pdf
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        // "raw" (integrated from velocity)
        public Angle rollPosition = Radians.of(0.0);
        public Angle pitchPosition = Radians.of(0.0);
        public Angle yawPosition = Radians.of(0.0);

        // "fused" (combined with magnetometer or accelerometer)
        public Angle rollFusedPosition = Radians.of(0.0);
        public Angle pitchFusedPosition = Radians.of(0.0);
        public Angle yawFusedPosition = Radians.of(0.0);

        // raw velocities
        public AngularVelocity rollVelocity = RadiansPerSecond.of(0.0);
        public AngularVelocity pitchVelocity = RadiansPerSecond.of(0.0);
        public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }
}