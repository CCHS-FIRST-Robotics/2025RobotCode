package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final DigitalInput irSensor = new DigitalInput(Constants.CORAL_SENSOR_PORT);
    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    public Coral(CoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);
    }

    // to intake coral: 
    //  press button, check if ir sensor beam is broken, open claw, then move the arm down, set wrist position, close claw, then swing to L4 (by default)

    // to set arm position: 
    //  press button, move the arm, set wrist position

    // to outtake: 
    //  press button, open the claw

    // to knock algae off: 
    //  press button, set arm to some position, drive forward, move it to some other position
}