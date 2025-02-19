package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import edu.wpi.first.units.measure.Angle;


public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs{

        public int tagId = -1;
        public Angle tagAngle = Degrees.of(-1);

        //public final ArrayList<Double> tagData = new ArrayList<Double>();

    }

    //Updates loggable inputs
    public default void updateInputs(CameraIOInputs inputs) {}
}
