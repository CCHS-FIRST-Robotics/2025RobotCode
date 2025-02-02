package frc.robot.subsystems.algaIO;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaIO {
    @AutoLog
    public static class AlgaIOInputs {
        public double motorVoltage;
        public double motorCurrent;
        public double motorTemperature;
    }

    public default void setVoltage(Voltage volts) {}

    public default void updateInputs(AlgaIOInputs inputs) {}
}