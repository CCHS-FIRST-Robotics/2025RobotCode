package frc.robot.subsystems.coralIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorVoltage;
        public double motorCurrent;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Voltage volts) {}

    public default void setPosition(Angle position){}

    public default void updateInputs(ElevatorIOInputs inputs) {}
}