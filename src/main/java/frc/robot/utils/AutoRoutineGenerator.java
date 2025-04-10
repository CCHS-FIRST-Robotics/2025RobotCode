package frc.robot.utils;

import choreo.auto.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;

public class AutoRoutineGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final CoralCommandCompositer coralCommandCompositer;

    public AutoRoutineGenerator(
        Drive drive,
        PoseEstimator poseEstimator,
        CoralCommandCompositer coralCommandCompositer
    ) {
        autoFactory = new AutoFactory(
            poseEstimator::getPose,
            poseEstimator::resetPosition,
            drive::runAutoPosition,
            false,
            drive
        );

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.coralCommandCompositer = coralCommandCompositer;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory = routine.trajectory("Test");

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory.resetOdometry()
            .andThen(trajectory.cmd())
            .andThen(new DriveWithPosition(drive, poseEstimator, trajectory.getFinalPose().get()))
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUp() {
        return new InstantCommand(() -> poseEstimator.resetPosition(new Pose2d()))
        .andThen(new DriveWithPosition(drive, poseEstimator, new Pose2d(-2, 0, new Rotation2d())));
    }

    
    
    public AutoRoutine oneCoralL4() {
        AutoRoutine routine = autoFactory.newRoutine("1CoralL4");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("1CoralL4", 0);

        // when routine begins, reset odometry, start first trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );
        
        trajectory0.done().onTrue(
            // hold the position
            new DriveWithPosition(drive, poseEstimator, trajectory0.getFinalPose().get())
            .alongWith(
                // prep the coral
                coralCommandCompositer.prepL4WithWait()
            )
            // align with the apriltag
            // .andThen(new DriveWithApriltag(
            //     drive, 
            //     poseEstimator, 
            //     17, 
            //     PhysicalConstants.DrivePositions.L4, 
            //     true
            // ))
            // run coral
            .andThen(coralCommandCompositer.runL4())
        );

        return routine;
    }
}