package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    public double driveKv = 0.136898; // ! tf
    public double driveKa = 0.020864;

    @AutoLog
    public static class ModuleIOInputs {
        public Angle driveRawPositionRad = Radians.of(0.0);
        public Angle drivePositionRad = Radians.of(0.0);
        public AngularVelocity driveVelocityRadPerSec = RadiansPerSecond.of(0.0);
        public Voltage driveAppliedVolts = Volts.of(0.0);
        public Voltage driveAverageBusVoltage = Volts.of(12);
        public Current driveCurrentAmps = Amps.of(0);
        public Temperature driveTempCelcius = Celsius.of(0);

        public Angle turnAbsolutePositionRad = Radians.of(0.0);
        public Angle turnPositionRad = Radians.of(0.0);
        public AngularVelocity turnVelocityRadPerSec = RadiansPerSecond.of(0.0);
        public Voltage turnAppliedVolts = Volts.of(0.0);
        public Voltage turnAverageBusVoltage = Volts.of(12);
        public Current turnCurrentAmps = Amps.of(0.0);
        public Temperature turnTempCelcius = Celsius.of(0.0);
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(Voltage volts) {}

    public default void setTurnVoltage(Voltage volts) {}

    public default void setDriveVelocity(AngularVelocity velocityRadPerSec) {}

    public default void setTurnPosition(Angle positionRad) {}

    public default void setDriveBrakeMode(boolean enable) {}

    public default void setTurnBrakeMode(boolean enable) {}
}