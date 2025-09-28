package frc.robot.subsystems.drive;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog // uses doubles so that units are clear in advantagescope
    public static class ModuleIOInputs {
        public double driveVoltage;
        public double driveCurrent;
        public double drivePosition; // radians
        public double driveVelocity; // radians per second
        public double driveTemperature;

        public double turnVoltage;
        public double turnCurrent;
        public double turnPosition;
        public double turnVelocity;
        public double turnTemperature;
    }

    public default void setDriveVoltage(Voltage volts) {}

    public default void setTurnVoltage(Voltage volts) {}

    public default void setDriveVelocity(AngularVelocity velocityRadPerSec) {}

    public default void setTurnPosition(Angle positionRad) {}

    public default void updateInputs(ModuleIOInputs inputs) {}
}