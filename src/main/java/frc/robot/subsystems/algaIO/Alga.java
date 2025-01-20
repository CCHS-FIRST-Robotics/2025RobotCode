package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Alga extends SubsystemBase {
    private final AlgaIO io;
    DigitalInput irSensor = new DigitalInput(0);
    private final AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();

    public Alga(AlgaIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("algaIO", inputs);
    }

    public void start(Voltage volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(Volts.of(0));
    }

    // intake until the beam is broken
    public Command getIntakeCommand() {
        return startEnd(() -> start(Volts.of(8)), this::stop).until(() -> irSensor.get());
    }

    // output until the beam isn't broken
    public Command getOutputCommand() {
        return startEnd(() -> start(Volts.of(-8)), this::stop).until(() -> !irSensor.get());
    }
}