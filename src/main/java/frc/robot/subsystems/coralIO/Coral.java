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
import frc.robot.constants.PhysicalConstants.CoralPositions;

public class Coral extends SubsystemBase {
    private final CoralIO io;
    private final DigitalInput troughSwitch; // ! might become an ir sensor
    private final CoralIOInputs inputs = new CoralIOInputs();

    private Angle wristPosition = Rotations.of(0);
    private boolean clawPosition = false;

    private final SysIdRoutine elevatorSysIdRoutine;
    private final SysIdRoutine armSysIdRoutine;
    private final SysIdRoutine wristSysIDRoutine;
    private final SysIdRoutine clawSysIDRoutine;

    public Coral(CoralIO io, int troughPort) {
        this.io = io;

        troughSwitch = new DigitalInput(troughPort);

        elevatorSysIdRoutine = sysIdRoutineFactory(
            "elevator", 
            (volts) -> io.setElevatorVoltage(volts), 
            Volts.per(Second).of(1), Volts.of(3), Seconds.of(7)
        );
        armSysIdRoutine = sysIdRoutineFactory(
            "arm", 
            (volts) -> io.setArmVoltage(volts), 
            Volts.per(Second).of(1), Volts.of(3), Seconds.of(5)
        );
        wristSysIDRoutine = sysIdRoutineFactory(
            "wrist", 
            (volts) -> io.setWristVoltage(volts), 
            Volts.per(Second).of(1), Volts.of(5), Seconds.of(7)
        );
        clawSysIDRoutine = sysIdRoutineFactory(
            "claw", 
            (volts) -> io.setClawVoltage(volts), 
            Volts.per(Second).of(1), Volts.of(5), Seconds.of(7)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("coralIO", inputs);

        Logger.recordOutput("outputs/coral/troughSwitch", troughSwitch.get());

        io.setWristPosition(wristPosition);
        io.setClawPosition(clawPosition);
    }

    // ————— testing command factories ————— //

    public Command getSetElevatorCommand(Angle angle){
        return new InstantCommand(() -> io.setElevatorPosition(angle));
    }

    public Command getSetArmCommand(Angle angle){
        return new InstantCommand(() -> io.setArmPosition(angle));
    }

    public void setWristPosition(Angle angle){
        wristPosition = angle;
    }

    public void setClawPosition(boolean open){
        clawPosition = open;
    }

    // ————— final command factories ————— // 

    // open claw, then move the elevator and arm down while setting wrist position
    public Command getPrepIntakeCommand(){
        return new InstantCommand(() -> setClawPosition(true))
        .andThen(
            new InstantCommand(() -> io.setElevatorPosition(CoralPositions.INTAKE.elevatorPosition.getValue()))
            .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.INTAKE.armPosition)))
            .alongWith(new InstantCommand(() -> io.setWristPosition(CoralPositions.INTAKE.wristPosition)))
        );
    }
    
    // check if ir sensor beam is broken, close claw, then swing to L4
    public Command getIntakeCommand(){
        if(!troughSwitch.get()){
            return null;
        }
        return new InstantCommand(() -> setClawPosition(false))
        .andThen(
            new InstantCommand(() -> io.setElevatorPosition(CoralPositions.L4.elevatorPosition.getValue()))
            .alongWith(new InstantCommand(() -> io.setArmPosition(CoralPositions.L4.armPosition)))
        );
    }

    // move elevator and arm, set wrist position
    public Command getSetCoralIOPositionCommand(CoralPositions.CoralPosition position){
        return new InstantCommand(() -> io.setElevatorPosition(position.elevatorPosition.getValue()))
        .alongWith(new InstantCommand(() -> io.setArmPosition(position.armPosition)))
        .alongWith(new InstantCommand(() -> io.setWristPosition(position.wristPosition)));
    }

    // press button, open the claw
    public Command getOuttakeCommand(){
        return  new InstantCommand(() -> setClawPosition(true));
    }

    // press button, set arm to some position, drive forward, move it to some other position
    public Command getDealgifyComamand(){
        return null; // ! code this at some point
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

    public Command wristSysIdFull(){
        return wristSysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(wristSysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(wristSysIDRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(wristSysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command clawSysIdFull(){
        return clawSysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(clawSysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(clawSysIDRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(clawSysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}