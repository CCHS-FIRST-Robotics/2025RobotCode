package frc.robot.subsystems.algaeIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Alga extends SubsystemBase {
    AlgaIO io;
    // private double startTime; // makes sure that the initial current spike from turning on the motor is ignored //! could be really problematic actually
    AlgaIOInputsAutoLogged inputs = new AlgaIOInputsAutoLogged();

    public Alga(AlgaIO io) {
        this.io = io;
    }

    public void start(Voltage volts) {
        io.setVoltage(volts);
        // startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        io.setVoltage(Volts.of(0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("algaIO", inputs);
    }

    public boolean detectNote() {
        // return inputs.motorCurrent > 28 && Timer.getFPGATimestamp() - startTime > 0.1; // ! magic numbers
        return false;
        // ! honestly just use a fucking ir sensor
    }

    // turns motor on until note detected
    public Command getIntakeCommand() {
        return startEnd(() -> start(Volts.of(8)), this::stop).until(() -> detectNote());
    }

    public Command getOutputCommand() {
        return startEnd(() -> start(Volts.of(-8)), this::stop).until(() -> detectNote());
    }
}