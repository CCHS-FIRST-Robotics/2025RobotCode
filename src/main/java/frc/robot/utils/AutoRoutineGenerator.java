package frc.robot.utils;

import choreo.auto.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class AutoRoutineGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    public AutoRoutineGenerator(
        Drive drive,
        PoseEstimator poseEstimator
    ) {
        autoFactory = new AutoFactory(
            poseEstimator::getPose,
            poseEstimator::resetPosition,
            drive::runAutoPosition,
            DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Blue : false,
            drive
        );

        this.drive = drive;
        this.poseEstimator = poseEstimator;
    }

    public Command backUp() {
        return new InstantCommand(() -> poseEstimator.resetPosition(new Pose2d()))
        .andThen(new DriveWithPosition(drive, poseEstimator, new Pose2d(-2, 0, new Rotation2d())));
    }

    public AutoRoutine twoMeter() {
        AutoRoutine routine = autoFactory.newRoutine("2Meter");

        // load trajectories
        AutoTrajectory trajectory = routine.trajectory("2Meter");

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory.resetOdometry()
            .andThen(trajectory.cmd())
            .andThen(new DriveWithPosition(drive, poseEstimator, trajectory.getFinalPose().get()))
            .andThen(new DriveWithPosition(drive, poseEstimator, new Pose2d(0, 0, new Rotation2d(Math.PI))))
        );

        return routine;
    }
    
    // ! in the future put preloaded coral on the reef, get another coral from the station, put that on the reef too
    public AutoRoutine twoCoralChoreo() {
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