package frc.robot.subsystems.algaIO;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaIO {
    @AutoLog
    public static class AlgaIOInputs {
        public double algaVoltage;
        public double algaCurrent;
        public double algaTemperature;

        public double drawbridgeVoltage;
        public double drawbridgeCurrent;
        public double drawbridgeTemperature;
    }

    public default void setAlgaVoltage(Voltage volts) {}

    public default void setDrawbridgeVoltage(Voltage volts) {}

    public default void updateInputs(AlgaIOInputs inputs) {}
}