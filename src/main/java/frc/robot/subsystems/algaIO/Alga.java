package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

public class Alga extends SubsystemBase {
    private final AlgaIO io;
    private final DigitalInput upSwitch;
    private final DigitalInput downSwitch;
    private final Timer timer;
    private final AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();

    public Alga(AlgaIO io, int upPort, int downPort) {
        this.io = io;
        upSwitch = new DigitalInput(upPort);
        downSwitch = new DigitalInput(downPort);
        timer = new Timer();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("algaIO", inputs);

        Logger.recordOutput("outputs/alga/upSwitch", upSwitch.get());
        Logger.recordOutput("outputs/alga/downSwitch", downSwitch.get());
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
        return inputs.algaCurrent < 7;
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

    public void stayUp(){
        io.setDrawbridgeVoltage(Volts.of(0.2));
    }

    public void stayDown(){
        io.setDrawbridgeVoltage(Volts.of(0));
    }

    public boolean drawbridgeUp(){
        return !upSwitch.get();
    }

    public boolean drawbridgeDown(){
        return !downSwitch.get();
    }

    public Command getDownAtMatchStartCommand(){
        return new InstantCommand(() -> timer.start())
            .andThen(startEnd(this::up, this::stayUp).until(() -> timer.get() > 0.5)) // ! maybe 0.75
            .andThen(this.getDownCommand());
    }

    public Command getUpCommand() {
        return startEnd(this::up, this::stayUp).until(this::drawbridgeUp);
    }

    public Command getDownCommand() {
        return startEnd(this::down, this::stayDown).until(this::drawbridgeDown);
    }
}