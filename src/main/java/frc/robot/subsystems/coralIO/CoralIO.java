package frc.robot.subsystems.coralIO;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs {
        public double motorVoltage;
        public double motorCurrent;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Voltage volts) {}

    public default void updateInputs(CoralIOInputs inputs) {}
}