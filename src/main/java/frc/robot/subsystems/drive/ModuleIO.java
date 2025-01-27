package frc.robot.subsystems.drive;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVoltage;
        public double driveCurrent;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveTemperature;

        public double turnAppliedVolts;
        public double turnAverageBusVoltage;
        public double turnAbsolutePositionRad;
        public double turnPositionRad;
        public double turnVelocityRadPerSec;
        public double turnCurrentAmps;
        public double turnTempCelcius;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(Voltage volts) {}

    public default void setTurnVoltage(Voltage volts) {}

    public default void setDriveVelocity(AngularVelocity velocityRadPerSec) {}

    public default void setTurnPosition(Angle positionRad) {}
}