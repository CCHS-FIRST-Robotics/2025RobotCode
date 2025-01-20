package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    public Coral(CoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);
    }

    public void start(Voltage volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(Volts.of(0));
    }

    public boolean detectNote() {
        return false;
    }

    public Command getIntakeCommand() {
        return startEnd(() -> start(Volts.of(8)), this::stop).until(() -> detectNote());
    }

    public Command getOutputCommand() {
        return startEnd(() -> start(Volts.of(-8)), this::stop).until(() -> detectNote());
    }
}