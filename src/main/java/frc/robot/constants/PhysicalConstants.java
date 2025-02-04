package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.AbstractMap.SimpleEntry;

public final class PhysicalConstants {
    // ————— swerve constants ————— // 
    // ! recalculate these
    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(2); // 4.5
    public static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(3); // 9
    public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI); // 8 * pi
    public static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Seconds).of(2 * Math.PI); // 10 * pi
    
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

    public static final double DRIVE_AFTER_ENCODER_REDUCTION = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_AFTER_ENCODER_REDUCTION = 150.0 / 7.0;
    public static final double COUPLING_RATIO = 50.0 / 14.0;

    // ————— coral constants ————— //
    public static class CoralPosition{ // coralIO position struct
        public final SimpleEntry<Distance, Angle> elevatorPosition;
        public final Angle armPosition;
        public final Angle wristPosition;

        public CoralPosition(
            Distance elevatorDistance, 
            Angle elevatorAngle, 
            Angle armAngle, 
            Angle wristPosition
        ){
            this.elevatorPosition = new SimpleEntry<Distance, Angle>(elevatorDistance, elevatorAngle);
            this.armPosition = armAngle;
            this.wristPosition = wristPosition;
        }
    }

    public static final CoralPosition INTAKE = new CoralPosition(
        Inches.of(0), // elevator height
        Rotations.of(0), // elevator angle
        Rotations.of(0), // arm angle
        Rotations.of(0) // wrist angle
    );

    public static final CoralPosition L1 = new CoralPosition(
        Inches.of(18), 
        Rotations.of(0),
        Rotations.of(0),
        Rotations.of(0)
    ); 

    public static final CoralPosition L2 = new CoralPosition(
        Inches.of(31.875), 
        Rotations.of(0),
        Rotations.of(0),
        Rotations.of(0)
    ); 

    public static final CoralPosition L3 = new CoralPosition(
        Inches.of(47.625), 
        Rotations.of(0),
        Rotations.of(0),
        Rotations.of(0)
    ); 
    
    public static final CoralPosition L4 = new CoralPosition(
        Inches.of(72), 
        Rotations.of(0),
        Rotations.of(0),
        Rotations.of(0)
    );
}