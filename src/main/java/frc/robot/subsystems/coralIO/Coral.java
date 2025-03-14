package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.coralIO.CoralIO.CoralIOInputs;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final DigitalInput troughSensor; // ! might become an ir sensor
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

    // ————— testing command factories ————— //

    public Command getSetElevatorVoltageCommand(Voltage volts){
        return new InstantCommand(() -> io.setElevatorVoltage(volts));
    }

    public Command getSetElevatorCommand(Angle angle){
        return new InstantCommand(() -> io.setElevatorPosition(angle));
    }

    public Command getSetArmVoltageCommand(Voltage volts){
        return new InstantCommand(() -> io.setArmVoltage(volts));
    }

    public Command getSetArmCommand(Angle angle){
        return new InstantCommand(() -> io.setArmPosition(angle));
    }

    // ————— final command factories ————— // 

    // // open claw, then move the elevator and arm down while setting wrist position
    // public Command getPrepIntakeCommand(){
    //     return new InstantCommand(() -> setClawPosition(true))
    //     .andThen(
    //         new InstantCommand(() -> io.setElevatorPosition(CoralPositions.INTAKE.elevatorPosition.getValue()))
    //         .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.INTAKE.armPosition)))
    //         .alongWith(new InstantCommand(() -> io.setWristPosition(CoralPositions.INTAKE.wristPosition)))
    //     );
    // }
    
    // // check if ir sensor beam is broken, close claw, then swing to L4
    // public Command getIntakeCommand(){
    //     if (troughSensor.get()) {
    //         return null;
    //     }
    //     return new InstantCommand(() -> setClawPosition(false))
    //     .andThen(
    //         new InstantCommand(() -> io.setElevatorPosition(CoralPositions.L4.elevatorPosition.getValue()))
    //         .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.L4.armPosition)))
    //         .alongWith(new InstantCommand(() -> io.setWristPosition(CoralPositions.L4.wristPosition)))
    //     );
    // }

    // move elevator and arm
    public Command getSetCoralPositionCommand(Angle[] position){
        return new InstantCommand(() -> io.setElevatorPosition(position[0]))
        .alongWith(new InstantCommand(() -> io.setArmPosition(position[1])));
    }

    // ————— sysid command factories ————— //

    public SysIdRoutine sysIdRoutineFactory(
        String joint,
        Consumer<Voltage> voltageConsumer,
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeOut
     ){
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

    public Command elevatorSysIdFull(){
        return elevatorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(elevatorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(elevatorSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(elevatorSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command armSysIdFull(){
        return armSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(armSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(armSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(armSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}