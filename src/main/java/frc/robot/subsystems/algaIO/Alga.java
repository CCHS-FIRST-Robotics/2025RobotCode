package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.VirtualConstants;

public class Alga extends SubsystemBase {
    private final AlgaIO io;
    private final DigitalInput irSensor = new DigitalInput(VirtualConstants.ALGA_SENSOR_PORT);
    private final AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();

    public Alga(AlgaIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("algaIO", inputs);
    }

    public void in() {
        io.setVoltage(Volts.of(-2));
    }

    public void hold() {
        io.setVoltage(Volts.of(-0.4));
    }

    public void out() {
        io.setVoltage(Volts.of(4)); 
    }
    
    public void stop() {
        io.setVoltage(Volts.of(0));
    }

    public boolean algaIn() {
        return inputs.motorCurrent > 30; // ! add ir sensor as well, both is good
    }

    public boolean algaOut() {
        return inputs.motorCurrent < 3;
    }

    // intake until the beam is broken
    public Command getIntakeCommand() {
        return startEnd(this::in, this::hold).until(this::algaIn);
    }

    // output until the beam isn't broken
    public Command getOutputCommand() {
        return startEnd(this::out, this::stop).until(this::algaOut);
    }
}