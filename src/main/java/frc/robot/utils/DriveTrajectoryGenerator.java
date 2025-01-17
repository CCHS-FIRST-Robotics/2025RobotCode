package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.*;
import java.util.ArrayList;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

public final class DriveTrajectoryGenerator {
    public static DriveTrajectory generateChoreoTrajectory(String path) {
        Optional<Trajectory<SwerveSample>> choreoTrajectory = Choreo.loadTrajectory(path);

        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();

        for (int i = 0; i < (int) (choreoTrajectory.get().getTotalTime() / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;
            SwerveSample state = choreoTrajectory.get().sampleAt(time, DriverStation.getAlliance().get() == DriverStation.Alliance.Red).get();
            poseTrajectory.add(new Pose2d(state.x, state.y, new Rotation2d(state.heading)));
            velocityTrajectory.add(new Twist2d(state.vx, state.vy, state.omega));
        }

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }
}