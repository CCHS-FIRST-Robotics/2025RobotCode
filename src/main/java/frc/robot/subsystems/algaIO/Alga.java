package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.VirtualConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Alga extends SubsystemBase {
    private final AlgaIO io;
    private final DigitalInput irSensor = new DigitalInput(VirtualConstants.ALGA_SENSOR_PORT);
    private final AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();

    Voltage IOVolts = Volts.of(8);

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
        return startEnd(() -> start(IOVolts), this::stop).until(() -> irSensor.get());
    }

    // output until the beam isn't broken
    public Command getOutputCommand() {
        return startEnd(() -> start(IOVolts.times(-1)), this::stop).until(() -> !irSensor.get());
    }
}