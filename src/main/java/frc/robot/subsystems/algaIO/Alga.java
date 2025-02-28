package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

public class Alga extends SubsystemBase {
    private final AlgaIO io;
    private final AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();
    // ! also limit switches for drawbridge

    public Alga(AlgaIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("algaIO", inputs);
    }

    // ————— alga ————— //

    public void in() {
        io.setAlgaVoltage(Volts.of(2));
    }

    public void hold() {
        io.setAlgaVoltage(Volts.of(0.4));
    }

    public void out() {
        io.setAlgaVoltage(Volts.of(-4));
    }
    
    public void stop() {
        io.setAlgaVoltage(Volts.of(0));
    }

    public boolean algaIn() {
        return inputs.algaCurrent > 30;
    }

    public boolean algaOut() {
        return inputs.algaCurrent < 3;
    }

    public Command getIntakeCommand() {
        return startEnd(this::in, this::hold).until(this::algaIn);
    }

    public Command getOutputCommand() {
        return startEnd(this::out, this::stop).until(this::algaOut);
    }

    // ————— drawbridge ————— //

    public void up(){
        io.setDrawbridgeVoltage(Volts.of(1));
    }

    public void down(){
        io.setDrawbridgeVoltage(Volts.of(-1));
    }

    public void stay(){
        io.setDrawbridgeVoltage(Volts.of(0)); // ! bad naming; also probably should be greater than 0?
    }

    public boolean drawbridgeUp(){
        return false;
    }

    public boolean drawbridgeDown(){
        return false;
    }

    public Command getUpCommand() {
        return startEnd(this::up, this::hold).until(this::drawbridgeUp);
    }

    public Command getDownCommand() {
        return startEnd(this::out, this::stop).until(this::drawbridgeDown);
    }
}