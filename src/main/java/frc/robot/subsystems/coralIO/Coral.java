package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.*;
import frc.robot.constants.PhysicalConstants.*;

public class Coral extends SubsystemBase {
    // positive voltage is down for the elevator, maybe invert it
    private final CoralIO io;
    private final DigitalInput troughSensor = new DigitalInput(VirtualConstants.TROUGH_SENSOR_PORT);
    private final DigitalInput elevatorSwitch1 = new DigitalInput(VirtualConstants.ELEVATOR_SWITCH_1_PORT);
    private final DigitalInput elevatorSwitch2 = new DigitalInput(VirtualConstants.ELEVATOR_SWITCH_2_PORT);
    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    // ! the arm can only rotate above the elevator

    public Coral(CoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);
    }

    public Command getElevatorUpCommand(){
        // return new StartEndCommand(
        //     () -> io.setElevatorVoltage(Volts.of(2)), 
        //     () -> io.setElevatorVoltage(Volts.of(0))
        //     // .until(() -> elevatorSwitch1.get()
        // );
        return new InstantCommand(() -> io.setElevatorVoltage(Volts.of(2)));
    }

    public Command getElevatorDownCommand(){
        // return new StartEndCommand(
        //     () -> io.setElevatorVoltage(Volts.of(-2)), 
        //     () -> io.setElevatorVoltage(Volts.of(0))
        //     // .until(() -> elevatorSwitch2.get()
        // );
        return new InstantCommand(() -> io.setElevatorVoltage(Volts.of(-2)));
    }

    public Command getStopElevatorCommand(){
        return new InstantCommand(() -> io.setElevatorVoltage(Volts.of(0)));
    }

    public Command getArmOnCommand(){
        return new InstantCommand(() -> io.setArmVoltage(Volts.of(2)));
    }

    // open claw, then move the elevator and arm down while setting wrist position
    public Command getPrepIntakeCommand(){
        return new InstantCommand(() -> io.setClawPosition(true))
            .andThen(
                new InstantCommand(() -> io.setElevatorPosition(PhysicalConstants.INTAKE.elevatorPosition.getValue()))
                .alongWith(new InstantCommand(() -> io.setArmPosition(PhysicalConstants.INTAKE.armPosition)))
                .alongWith(new InstantCommand(() -> io.setWristPosition(PhysicalConstants.INTAKE.wristPosition)))
            );
    }
    
    // check if ir sensor beam is broken, close claw, then swing to L4
    public Command getIntakeCommand(){
        if(!troughSensor.get()){
            return null; // ! idk if this'll throw an error
        }

        return new InstantCommand(() -> io.setClawPosition(false))
            .andThen(
                new InstantCommand(() -> io.setElevatorPosition(PhysicalConstants.L4.elevatorPosition.getValue()))
                .alongWith(new InstantCommand(() -> io.setArmPosition(PhysicalConstants.L4.armPosition)))
            );
    }

    // move elevator and arm, set wrist position
    public Command getSetCoralIOPositionCommand(CoralPosition position){
        return new InstantCommand(() -> io.setElevatorPosition(position.elevatorPosition.getValue()))
            .alongWith(new InstantCommand(() -> io.setArmPosition(position.armPosition)))
            .alongWith(new InstantCommand(() -> io.setWristPosition(position.wristPosition)));
    }

    // press button, open the claw
    public Command getOuttakeCommand(){
        return new InstantCommand(() -> io.setClawPosition(true));
    }

    // press button, set arm to some position, drive forward, move it to some other position
    public Command getDealgifyComamand(){
        return null; // ! code this at some point
    }
}