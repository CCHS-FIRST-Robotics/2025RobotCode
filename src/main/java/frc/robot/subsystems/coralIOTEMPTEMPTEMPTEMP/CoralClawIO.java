package frc.robot.subsystems.coralIOTEMPTEMPTEMPTEMP;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface CoralClawIO {
    @AutoLog
    public static class CoralClawIOInputs {
        public double motorVoltage;
        public double motorCurrent;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Voltage volts) {}

    public default void setPosition(Angle position) {}

    public default void updateInputs(CoralClawIOInputs inputs) {}
}