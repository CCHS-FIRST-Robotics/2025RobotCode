package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        public Angle rollPosition = Radians.of(0.0);
        public Angle pitchPosition = Radians.of(0.0);
        public Angle yawPosition = Radians.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}