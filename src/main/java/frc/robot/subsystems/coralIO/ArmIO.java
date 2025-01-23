package frc.robot.subsystems.coralIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
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

    public default void setArmPosition(Angle position){}

    public default void setWristPosition(Angle position){}

    public default void setClawPosition(Angle position){}

    public default void updateInputs(ArmIOInputs inputs) {}
}