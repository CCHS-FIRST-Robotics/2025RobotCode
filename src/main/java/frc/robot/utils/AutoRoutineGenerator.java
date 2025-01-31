package frc.robot.utils;

import choreo.auto.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutineGenerator {
    private final AutoFactory autoFactory;

    // ! it would be really painful to have to write what to do at each eventmarker for every single auto I write
    public AutoRoutineGenerator(
        Drive drive
    ) {
        autoFactory = new AutoFactory(
            drive::getPose,
            drive::resetPoseEstimator,
            drive::runPosition,
            DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false,
            drive
        );
    }
    
    // ! in the future put preloaded coral on the reef, get another coral from the station, put that on the reef too
    public AutoRoutine twoCoral() {
        AutoRoutine routine = autoFactory.newRoutine("2Coral");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("2Coral", 0);
        AutoTrajectory trajectory1 = routine.trajectory("2Coral", 1);
        AutoTrajectory trajectory2 = routine.trajectory("2Coral", 2);

        // when routine begins, reset odometry, start first trajectory, begin moving the elevator
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );
        
        // start the next trajectory
        trajectory0.done().onTrue(
            trajectory1.cmd()
        );

        // start the next trajectory
        trajectory1.done().onTrue(
            trajectory2.cmd()
        );

        return routine;
    }
}