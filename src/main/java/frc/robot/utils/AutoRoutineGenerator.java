package frc.robot.utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralIOTEMPTEMPTEMPTEMP.Coral;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.constants.HardwareConstants.*;

public class AutoRoutineGenerator {
    AutoFactory autoFactory;

    Drive drive;
    Elevator elevator;
    Coral coral;

    // it would be really painful to have to write what to do at each eventmarker for every single auto I write
    
    public AutoRoutineGenerator(
        Drive drive, 
        Elevator elevator, 
        Coral coral
    ) {
        autoFactory = new AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            drive::followTrajectory,
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            drive
        );

        this.drive = drive;
        this.elevator = elevator;
        this.coral = coral;
    }
    
    // put preloaded coral on the reef, get another coral from the station, put that on the reef too
    public AutoRoutine twoCoral() {
        AutoRoutine routine = autoFactory.newRoutine("2Coral");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("2Coral", 0);
        AutoTrajectory trajectory1 = routine.trajectory("2Coral", 1);
        AutoTrajectory trajectory2 = routine.trajectory("2Coral", 2);

        // when routine begins, reset odometry, start first trajectory, begin moving the elevator
        routine.active().onTrue(
            Commands.sequence(
                trajectory0.resetOdometry(), 
                trajectory0.cmd()
                .alongWith(elevator.getSetPositionCommand(ElevatorPosition.L1))
            )
        );
        // at eventmarker output1, run the coral output
        trajectory0.atTime("output1").onTrue(null);
        
        // start the next trajectory
        trajectory0.done().onTrue(
            trajectory1.cmd()
            .alongWith(elevator.getSetPositionCommand(ElevatorPosition.STATION))
        );
        // at eventmarker intake1, run the coral intake
        trajectory1.atTime("intake1").onTrue(null);

        // start the next trajectory
        trajectory1.done().onTrue(
            trajectory2.cmd()
            .alongWith(elevator.getSetPositionCommand(ElevatorPosition.L2))
        );
        // when the trajectory is done, run the coral output (instead of having an extra eventmarker)
        trajectory2.done().onTrue(null);

        return routine;
    }
}
