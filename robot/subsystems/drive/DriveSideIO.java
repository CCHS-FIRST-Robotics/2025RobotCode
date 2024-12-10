package frc.robot.subsystems.drive;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface DriveSideIO {
    @AutoLog
    public static class DriveSideIOInputs {
        public double motor1Current = 0; // amps
        public double motor1Voltage = 0; // volts
        public double motor1Temperature = 0; // celcius
        
        public double wheelPosition = 0; // rotations
        public double wheelVelocity = 0; // rotations per second
        public double motor1Position = 0; // rotations
        public double motor1Velocity = 0; // rotations per second

        public double currentSetpoint = 0; // rotations per second
        public double distanceTraveled = 0; // meters
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setVelocity(Measure<Velocity<Angle>> velocity) {}

    public default void updateInputs(DriveSideIOInputs inputs) {}
}