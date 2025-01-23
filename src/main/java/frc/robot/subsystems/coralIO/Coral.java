package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
    private final Elevator elevator;
    private final Arm arm;

    // will mainly hold command factories
    public Coral(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    public void periodic() {}

    // set position to station intake and then intake

    // set position to any level and then outtake

    // knock algae off
}