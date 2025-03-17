package frc.robot.subsystems.coralIO;


import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;
import edu.wpi.first.units.measure.*;

public interface CoralIO {
    public static class CoralIOInputs implements LoggableInputs {
        public double elevatorVoltage;
        public double elevatorCurrent;
        public double elevatorMotorPosition;
        public double elevatorMotorVelocity;
        public double elevatorEncoderPosition;
        public double elevatorEncoderVelocity;
        public double elevatorTemperature;

        public double armVoltage;
        public double armCurrent;
        public double armPosition;
        public double armVelocity;
        public double armAbsolutePosition;
        public double armAbsoluteVelocity;
        public double armTemperature;

        @Override
        public void toLog(LogTable table) {
            table.put("elevator/ElevatorVoltage", elevatorVoltage);
            table.put("elevator/ElevatorCurrent", elevatorCurrent);
            table.put("elevator/ElevatorPosition", elevatorMotorPosition);
            table.put("elevator/ElevatorVelocity", elevatorMotorVelocity);
            table.put("elevator/ElevatorAbsolutePosition", elevatorEncoderPosition);
            table.put("elevator/ElevatorAbsoluteVelocity", elevatorEncoderVelocity);
            table.put("elevator/ElevatorTemperature", elevatorTemperature);
            
            table.put("arm/ArmVoltage", armVoltage);
            table.put("arm/ArmCurrent", armCurrent);
            table.put("arm/ArmPosition", armPosition);
            table.put("arm/ArmVelocity", armVelocity);
            table.put("arm/ArmAbsolutePosition", armAbsolutePosition);
            table.put("arm/ArmAbsoluteVelocity", armAbsoluteVelocity);
            table.put("arm/ArmTemperature", armTemperature);
        }

        @Override
        public void fromLog(LogTable table) {
            elevatorVoltage = table.get("elevator/ElevatorVoltage", elevatorVoltage);
            elevatorCurrent = table.get("elevator/ElevatorCurrent", elevatorCurrent);
            elevatorMotorPosition = table.get("elevator/ElevatorPosition", elevatorMotorPosition);
            elevatorMotorVelocity = table.get("elevator/ElevatorVelocity", elevatorMotorVelocity);
            elevatorEncoderPosition = table.get("elevator/ElevatorAbsolutePosition", elevatorEncoderPosition);
            elevatorEncoderVelocity = table.get("elevator/ElevatorAbsoluteVelocity", elevatorEncoderVelocity);
            elevatorTemperature = table.get("elevator/ElevatorTemperature", elevatorTemperature);
            
            armVoltage = table.get("arm/ArmVoltage", armVoltage);
            armCurrent = table.get("arm/ArmCurrent", armCurrent);
            armPosition = table.get("arm/ArmPosition", armPosition);
            armVelocity = table.get("arm/ArmVelocity", armVelocity);
            armAbsolutePosition = table.get("arm/ArmAbsolutePosition", armAbsolutePosition);
            armAbsoluteVelocity = table.get("arm/ArmAbsoluteVelocity", armAbsoluteVelocity);
            armTemperature = table.get("arm/ArmTemperature", armTemperature);
        }
    }

    public default void setElevatorVoltage(Voltage volts) {}

    public default void setElevatorPosition(Angle position) {}

    public default boolean elevatorAtSetpoint() {
        return false;
    }

    public default void setArmVoltage(Voltage volts) {}

    public default void setArmPosition(Angle position) {}

    public default boolean armAtSetpoint() {
        return false;
    }
    
    public default void updateInputs(CoralIOInputs inputs) {}
}