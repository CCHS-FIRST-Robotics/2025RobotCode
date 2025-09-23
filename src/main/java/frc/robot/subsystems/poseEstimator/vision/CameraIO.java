package frc.robot.subsystems.poseEstimator.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected;
        public PoseDataEntry[] visionPoseData;
    }

    public default void updateInputs(CameraIOInputs inputs) {}

    public static record PoseDataEntry(Pose3d robotPose, double timestamp, Matrix<N3, N1> standardDeviation) {}
}