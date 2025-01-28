package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import java.util.*;
import java.util.Map.*;
import java.util.AbstractMap.SimpleEntry;

public class HardwareConstants {
    // swerve stats // ! recalculate these
    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.5);
    public static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(9.0);
    public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(8 * Math.PI);
    public static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Seconds).of(10 * Math.PI);
    
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final Distance TRACK_WIDTH_X = Inches.of(23); // shorter side // ! might be the wrong way
    public static final Distance TRACK_WIDTH_Y = Inches.of(27);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0)
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public static enum coralPosition{
        STATION,
        L1, 
        L2, 
        L3, 
        L4
    }

    // ! add rotation values
    public static final HashMap<coralPosition, Entry<Distance, Angle>> ELEVATOR_POSITIONS = new HashMap<coralPosition, Entry<Distance, Angle>>(
        Map.of( 
            coralPosition.L1, new SimpleEntry<Distance, Angle>(Inches.of(18), Rotations.of(0)), 
            coralPosition.L2, new SimpleEntry<Distance, Angle>(Inches.of(31.875), Rotations.of(0)),
            coralPosition.L3, new SimpleEntry<Distance, Angle>(Inches.of(47.625), Rotations.of(0)),
            coralPosition.L4, new SimpleEntry<Distance, Angle>(Inches.of(72), Rotations.of(0))
        )
    );

    public static class CoralPosition{
        SimpleEntry<Distance, Angle> elevatorPosition;
        Angle armPosition;
        boolean wristPosition;
        boolean clawPosition;

        public CoralPosition(
            Distance elevatorDistance, 
            Angle elevatorAngle, 
            Angle armAngle, 
            boolean wristPosition, 
            boolean clawPosition
        ){
            this.elevatorPosition = new SimpleEntry<Distance, Angle>(elevatorDistance, elevatorAngle);
            this.armPosition = armAngle;
            this.wristPosition = wristPosition;
            this.clawPosition = clawPosition;
        }
    }
}