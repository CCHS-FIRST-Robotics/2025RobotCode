package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import java.util.*;
import java.util.Map.*;
import java.util.AbstractMap.SimpleEntry;

public class HardwareConstants {
    // swerve stats
    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.5);
    public static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(9.0);
    public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(8 * Math.PI);
    public static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Seconds).of(10 * Math.PI);
    
    public static final Distance TRACK_WIDTH_X = Inches.of(22.5);
    public static final Distance TRACK_WIDTH_Y = Inches.of(22.5);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0)
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    // elevator
    public static enum ElevatorPosition{
        L1, 
        L2, 
        L3, 
        L4
    }

    // ! add rotation values
    public static final HashMap<ElevatorPosition, Entry<Distance, Angle>> ELEVATOR_POSITIONS = new HashMap<ElevatorPosition, Entry<Distance, Angle>>(
        Map.of(
            ElevatorPosition.L1, new SimpleEntry<Distance, Angle>(Inches.of(18), Rotations.of(0)), 
            ElevatorPosition.L2, new SimpleEntry<Distance, Angle>(Inches.of(31.875), Rotations.of(0)),
            ElevatorPosition.L3, new SimpleEntry<Distance, Angle>(Inches.of(47.625), Rotations.of(0)),
            ElevatorPosition.L4, new SimpleEntry<Distance, Angle>(Inches.of(72), Rotations.of(0))
        )
    );
}