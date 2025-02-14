package frc.robot.subsystems.coralIO;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.coralIO.CoralIO.CoralIOInputs;
import frc.robot.constants.*;
import frc.robot.constants.PhysicalConstants.CoralPositions;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    // private final DigitalInput elevatorSwitchBottom = new DigitalInput(VirtualConstants.ELEVATOR_SWITCH_BOTTOM_PORT); // bottom
    // private final DigitalInput elevatorSwitchTop = new DigitalInput(VirtualConstants.ELEVATOR_SWITCH_TOP_PORT); // top
    private final DigitalInput troughSensor = new DigitalInput(VirtualConstants.TROUGH_SENSOR_PORT);
    private final CoralIOInputs inputs = new CoralIOInputs();

    public Coral(CoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);
        
        // Logger.recordOutput("outputs/coral/elevatorSwitchBottom", elevatorSwitchBottom.get());
        // Logger.recordOutput("outputs/coral/elevatorSwitchTop", elevatorSwitchTop.get());
        Logger.recordOutput("outputs/coral/troughSensor", troughSensor.get());
    }

    // ————— testing command factories ————— //

    public Command getSetElevatorCommand(Angle angle){
        return new InstantCommand(() -> io.setElevatorPosition(angle));
    }

    public Command getSetArmCommand(Angle angle){
        return new InstantCommand(() -> io.setArmPosition(angle));
    }

    public Command getSetWristVoltageCommand(Voltage volts){
        return new InstantCommand(() -> io.setWristVoltage(volts));
    }

    // ————— final command factories ————— //

    // open claw, then move the elevator and arm down while setting wrist position
    public Command getPrepIntakeCommand(){
        return new InstantCommand(() -> io.setClawPosition(true))
        .andThen(
            new InstantCommand(() -> io.setElevatorPosition(CoralPositions.INTAKE.elevatorPosition.getValue()))
            .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.INTAKE.armPosition)))
            .alongWith(new InstantCommand(() -> io.setWristPosition(CoralPositions.INTAKE.wristPosition)))
        );
    }
    
    // check if ir sensor beam is broken, close claw, then swing to L4
    public Command getIntakeCommand(){
        if(!troughSensor.get()){
            return null; // ! idk if this'll throw an error
        }

        return new InstantCommand(() -> io.setClawPosition(false))
        .andThen(
            new InstantCommand(() -> io.setElevatorPosition(CoralPositions.L4.elevatorPosition.getValue()))
            .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.L4.armPosition)))
        );
    }

    // move elevator and arm, set wrist position
    public Command getSetCoralIOPositionCommand(CoralPositions.CoralPosition position){
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