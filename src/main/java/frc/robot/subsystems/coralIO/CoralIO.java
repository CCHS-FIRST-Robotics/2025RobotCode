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
        
        public double clawVoltage;
        public double clawCurrent;
        public double clawPosition;
        public double clawVelocity;
        public double clawTemperature;
    }

    public default void setElevatorPosition(Angle position){}

    public default void setArmPosition(Angle position){}

    public default void setWristPosition(Angle position){}

    public default void setClawPosition(boolean open){}

    public default void updateInputs(CoralIOInputs inputs) {}
}