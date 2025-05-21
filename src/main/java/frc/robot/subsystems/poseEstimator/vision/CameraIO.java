package frc.robot.subsystems.poseEstimator.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected;
        // public ArrayList<PoseDataEntry> visionPoseData;
    }

    public default void updateInputs(CameraIOInputs inputs) {}

    public static class PoseDataEntry {
        private final Pose3d robotPose;
        private final double timestamp;
        private final Matrix<N3, N1> standardDeviation;

        public PoseDataEntry(Pose3d robotPose, double timestamp, Matrix<N3, N1> standardDeviation) {
            this.robotPose = robotPose;
            this.timestamp = timestamp;
            this.standardDeviation = standardDeviation;
        }

        public Pose3d getRobotPose() {
            return robotPose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Matrix<N3, N1> getStandardDeviation() {
            return standardDeviation;
        }

        @Override
        public String toString() {
            return "[" + robotPose + ", timestamp=" + timestamp + ", stddev=" + standardDeviation + "]";
        }
    }
}