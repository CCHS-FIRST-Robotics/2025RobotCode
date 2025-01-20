package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

import frc.robot.HardwareConstants;
import frc.robot.HardwareConstants.*;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setVoltage(Voltage volts){
        io.setVoltage(volts);
    }

    public void setPosition(ElevatorPosition position){
        io.setPosition(HardwareConstants.ELEVATOR_POSITIONS.get(position).getValue());
    }

    // public boolean isAtGoal() {
    //     return Math.abs(getArmAngle().in(Degrees) - targetAngle.in(Degrees)) < .8;
    // }

    public void updateInputs(){
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);
    }
}