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
        public double armMotorPosition;
        public double armMotorVelocity;
        public double armEncoderPosition;
        public double armEncoderVelocity;
        public double armTemperature;

        @Override
        public void toLog(LogTable table) {
            table.put("elevator/ElevatorVoltage", elevatorVoltage);
            table.put("elevator/ElevatorCurrent", elevatorCurrent);
            table.put("elevator/ElevatorMotorPosition", elevatorMotorPosition);
            table.put("elevator/ElevatorMotorVelocity", elevatorMotorVelocity);
            table.put("elevator/ElevatorEncoderPosition", elevatorEncoderPosition);
            table.put("elevator/ElevatorEncoderVelocity", elevatorEncoderVelocity);
            table.put("elevator/ElevatorTemperature", elevatorTemperature);
            
            table.put("arm/ArmVoltage", armVoltage);
            table.put("arm/ArmCurrent", armCurrent);
            table.put("arm/ArmMotorPosition", armMotorPosition);
            table.put("arm/ArmMotorVelocity", armMotorVelocity);
            table.put("arm/ArmEncoderPosition", armEncoderPosition);
            table.put("arm/ArmEncoderVelocity", armEncoderVelocity);
            table.put("arm/ArmTemperature", armTemperature);
        }

        @Override
        public void fromLog(LogTable table) {
            elevatorVoltage = table.get("elevator/ElevatorVoltage", elevatorVoltage);
            elevatorCurrent = table.get("elevator/ElevatorCurrent", elevatorCurrent);
            elevatorMotorPosition = table.get("elevator/ElevatorMotorPosition", elevatorMotorPosition);
            elevatorMotorVelocity = table.get("elevator/ElevatorMotorVelocity", elevatorMotorVelocity);
            elevatorEncoderPosition = table.get("elevator/ElevatorEncoderPosition", elevatorEncoderPosition);
            elevatorEncoderVelocity = table.get("elevator/ElevatorEncoderVelocity", elevatorEncoderVelocity);
            elevatorTemperature = table.get("elevator/ElevatorTemperature", elevatorTemperature);
            
            armVoltage = table.get("arm/ArmVoltage", armVoltage);
            armCurrent = table.get("arm/ArmCurrent", armCurrent);
            armMotorPosition = table.get("arm/ArmMotorPosition", armMotorPosition);
            armMotorVelocity = table.get("arm/ArmMotorVelocity", armMotorVelocity);
            armEncoderPosition = table.get("arm/ArmEncoderPosition", armEncoderPosition);
            armEncoderVelocity = table.get("arm/ArmEncoderVelocity", armEncoderVelocity);
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