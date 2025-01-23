package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.HardwareConstants.*;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    ElevatorPosition targetPosition;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);
    }

    public void setPosition(ElevatorPosition position){
        io.setPosition(HardwareConstants.ELEVATOR_POSITIONS.get(position).getValue());
    }

    public boolean atGoal(){
        return Math.abs(
            inputs.motorPosition - HardwareConstants.ELEVATOR_POSITIONS.get(targetPosition).getValue().in(Rotations)
        ) < 0.5; // ! magic number, change this later
    }

    public Command getSetPositionCommand(ElevatorPosition position){
        return new InstantCommand(() -> setPosition(position)).until(() -> atGoal());
    }
}