package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import frc.robot.HardwareConstants;
import frc.robot.HardwareConstants.*;

public class Elevator {
    private final ElevatorIO io;
    ElevatorPosition targetPosition;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setPosition(ElevatorPosition position){
        io.setPosition(HardwareConstants.ELEVATOR_POSITIONS.get(position).getValue());
    }

    public boolean atGoal(){
        return Math.abs(
            inputs.motorPosition - HardwareConstants.ELEVATOR_POSITIONS.get(targetPosition).getValue().in(Rotations)
        ) < 0.5; // ! magic number, change this later
    }

    public void updateInputs(){
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);
    }
}