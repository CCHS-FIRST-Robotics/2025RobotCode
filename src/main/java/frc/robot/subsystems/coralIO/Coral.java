package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

// ! honestly, why are arm and elevator even separate files?

public class Coral extends SubsystemBase {
    private final CoralIO io;

    public Coral(CoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {}

    // to intake coral: 
    //  press button, open claw, then move the arm down, set wrist position, close claw, then swing to L4 (by default)

    // to set arm position: 
    //  press button, move the arm, set wrist position

    // to outtake: 
    //  press button, open the claw

    // to knock algae off: 
    //  press button, set arm to some position, drive forward, move it to some other position
}