package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

//import frc.robot.constants.PhysicalConstants;
import frc.robot.subsystems.coralIO.CoralIO.CoralIOInputs;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final DigitalInput troughSensor;
    private final CoralIOInputs inputs = new CoralIOInputs();

    private final SysIdRoutine elevatorSysIdRoutine;
    private final SysIdRoutine armSysIdRoutine;

    public Coral(CoralIO io, int troughPort) {
        this.io = io;

        troughSensor = new DigitalInput(troughPort);

        elevatorSysIdRoutine = sysIdRoutineFactory(
            "elevator", 
            (volts) -> io.setElevatorVoltage(volts), 
            Volts.per(Second).of(1), Volts.of(3), Seconds.of(7)
        );
        armSysIdRoutine = sysIdRoutineFactory(
            "arm", 
            (volts) -> io.setArmVoltage(volts), 
            Volts.per(Second).of(0.25), Volts.of(0.5), Seconds.of(2)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);

        Logger.recordOutput("outputs/coral/troughSensor", troughSensor.get());
    }

    // ————— raw command factories ————— //

    public Command getSetElevatorVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setElevatorVoltage(volts));
    }

    public Command getSetElevatorCommand(Angle angle) {
        return runOnce(() -> io.setElevatorPosition(angle));
    }

    public Command getSetArmVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setArmVoltage(volts));
    }

    public Command getSetArmCommand(Angle angle) {
        return runOnce(() -> io.setArmPosition(angle));
    }

    // ————— processed command factories ————— // 

    public Command getSetCoralPositionCommand(Angle[] position) {
        return runOnce(() -> io.setElevatorPosition(position[0]))
        .alongWith(new InstantCommand(() -> io.setArmPosition(position[1])));
    }

    public Command getWaitUntilCoralInPositionCommand(Angle[] position) {
        return Commands.waitUntil(
            () -> (io.elevatorAtSetpoint(position[0]) && io.armAtSetpoint(position[1]))
        );
    }

    public Command getLowerArmWithVoltageCommand(Voltage volts, Angle armPositionThreshold) {
        return runOnce(() -> io.setArmVoltage(volts))
        .andThen(Commands.waitUntil(() -> inputs.armEncoderPosition <= armPositionThreshold.in(Rotations)));
    }

    public boolean troughSensesCoral() {
        return troughSensor.get(); // ! depends on how they wire it
    }

    // ————— sysid command factories ————— //

    public SysIdRoutine sysIdRoutineFactory(
        String joint,
        Consumer<Voltage> voltageConsumer,
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeOut
     ) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate, 
                stepVoltage, 
                timeOut, 
                (state) -> Logger.recordOutput("coral/" + joint + "/sysIdState", state.toString()) // send the data to advantagekit
            ),
            new SysIdRoutine.Mechanism(
                voltageConsumer,
                null, // no log consumer since advantagekit records the data
                this
            )
        );
    }

    public Command elevatorSysIdFull() {
        return elevatorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(elevatorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(elevatorSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(elevatorSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command armSysIdFull() {
        return armSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(armSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(armSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(armSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}