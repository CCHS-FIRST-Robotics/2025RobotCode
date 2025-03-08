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

        public double wristVoltage;
        public double wristCurrent;
        public double wristPosition;
        public double wristVelocity;
        public double wristAbsolutePosition;
        public double wristAbsoluteVelocity;
        public double wristTemperature;
        
        public double clawVoltage;
        public double clawCurrent;
        public double clawSwitch;
        public double clawTemperature;

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
            
            table.put("wrist/WristVoltage", wristVoltage);
            table.put("wrist/WristCurrent", wristCurrent);
            table.put("wrist/WristPosition", wristPosition);
            table.put("wrist/WristVelocity", wristVelocity);
            table.put("wrist/WristAbsolutePosition", wristAbsolutePosition);
            table.put("wrist/WristAbsoluteVelocity", wristAbsoluteVelocity);
            table.put("wrist/WristTemperature", wristTemperature);

            table.put("claw/ClawVoltage", clawVoltage);
            table.put("claw/ClawCurrent", clawCurrent);
            table.put("claw/ClawSwitch", clawSwitch);
            table.put("claw/ClawTemperature", clawTemperature);
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
            
            wristVoltage = table.get("wrist/WristVoltage", wristVoltage);
            wristCurrent = table.get("wrist/WristCurrent", wristCurrent);
            wristPosition = table.get("wrist/WristPosition", wristPosition);
            wristVelocity = table.get("wrist/WristVelocity", wristVelocity);
            wristAbsolutePosition = table.get("wrist/WristPosition", wristAbsolutePosition);
            wristAbsoluteVelocity = table.get("wrist/WristVelocity", wristAbsoluteVelocity);
            wristTemperature = table.get("wrist/WristTemperature", wristTemperature);
            
            clawVoltage = table.get("claw/ClawVoltage", clawVoltage);
            clawCurrent = table.get("claw/ClawCurrent", clawCurrent);
            clawSwitch = table.get("claw/ClawSwitch", clawSwitch);
            clawTemperature = table.get("claw/ClawTemperature", clawTemperature);
        }
    }

    public default void setElevatorVoltage(Voltage volts){}

    public default void setElevatorPosition(Angle position){}

    public default void setArmVoltage(Voltage volts){}

    public default void setArmPosition(Angle position){}

    public default void setWristVoltage(Voltage volts){}

    public default void setWristPosition(Angle position){}

    public default void setClawVoltage(Voltage volts){}

    public default void setClawPosition(boolean open){}
    
    public default void updateInputs(CoralIOInputs inputs) {}
}