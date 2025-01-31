package frc.robot.subsystems.coralIO;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs {
        // ! decide which ones of these to actually log
        public double elevatorVoltage;
        public double elevatorCurrent;
        public double elevatorPosition;
        public double elevatorVelocity;
        public double elevatorTemperature;

        public double armVoltage;
        public double armCurrent;
        public double armPosition;
        public double armVelocity;
        public double armTemperature;

        public double wristVoltage;
        public double wristCurrent;
        public double wristPosition;
        public double wristVelocity;
        public double wristTemperature;
        
        public boolean clawState;
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