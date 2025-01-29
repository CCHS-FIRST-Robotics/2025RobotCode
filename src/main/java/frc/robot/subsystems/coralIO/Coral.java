package frc.robot.subsystems.coralIO;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.VirtualConstants;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.PhysicalConstants.*;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final DigitalInput troughSensor = new DigitalInput(VirtualConstants.TROUGH_SENSOR_PORT);
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

    // open claw, then move the arm down while setting wrist position
    public Command getPrepIntakeCommand(){
        return new InstantCommand(() -> io.setClawPosition(true))
            .andThen(
                new InstantCommand(() -> io.setArmPosition(PhysicalConstants.INTAKE.armPosition))
                .alongWith(new InstantCommand(() -> io.setWristPosition(PhysicalConstants.INTAKE.wristPosition)))
            );
    }
    
    // check if ir sensor beam is broken, close claw, then swing to L4
    public Command getIntakeCommand(){
        if(!troughSensor.get()){
            return null; // ! idk if this'll throw an error
        }

        return new InstantCommand(() -> io.setClawPosition(false))
            .andThen(new InstantCommand(() -> io.setArmPosition(PhysicalConstants.L4.armPosition)));
        // ! maybe move the wrist too?
    }

    // move the arm, set wrist position
    public Command getSetArmPositionCommand(CoralPosition position){
        return new InstantCommand(() -> io.setArmPosition(position.armPosition))
            .andThen(new InstantCommand(() -> io.setWristPosition(position.wristPosition)));
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