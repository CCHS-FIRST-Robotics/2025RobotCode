package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;

import frc.robot.utils.AprilTag;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;


public interface CameraIO {
    public static class CameraIOInputs implements LoggableInputs {

        public int closestTagId = -1;
        public Measure<Distance> closestTagXdist = Meters.of(-1);
        public Measure<Distance> closestTagYdist = Meters.of(-1);
        public Measure<Angle> closestTagHeading = Degrees.of(-1);

        // Values for all tags found by the camera
        int numTags = 0;
        public ArrayList<AprilTag> tags = new ArrayList<AprilTag>();
        /*
         * IMPLEMENTS LOGGABLE INPUTS MANUALLY (NOT AUTOLOG) TO LOG CUSTOM AprilTag OBJECTS
         */
        @Override
        public void toLog(LogTable table) {
            table.put("closestTag/Id", closestTagId);
            table.put("closestTag/X", closestTagXdist);
            table.put("closestTag/Y", closestTagYdist);
            table.put("closestTag/Heading", closestTagHeading);

            table.put("numTags", tags.size());

            for (int i = 0; i < tags.size(); i++) {
                AprilTag tag = tags.get(i);
                table.put("tag" + i + "/Id", tag.getId());
                table.put("tag" + i + "/Distance", tag.getDistance());
            }
        }

        @Override
        public void fromLog(LogTable table) {
            closestTagId = table.get("closestTag/Id", closestTagId);
            closestTagXdist = table.get("closestTag/X", closestTagXdist);
            closestTagYdist = table.get("closestTag/Y", closestTagYdist);
            closestTagHeading = table.get("closestTag/Heading", closestTagHeading);

            numTags = table.get("numTags", numTags);
            for (int i = 0; i < numTags; i++) {
                int id = table.get("tag" + i + "/Id", -1);
                double xDist = table.get("tag" + i + "/xDist", -1);
                double yDist = table.get("tag" + i + "/yDist", -1);
                double heading = table.get("tag" + i + "/heading", -1);

                tags.add(new AprilTag(id, xDist, yDist, heading));
            }

        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(CameraIOInputs inputs) {}
}