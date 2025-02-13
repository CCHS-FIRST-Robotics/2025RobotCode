package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.AbstractMap.SimpleEntry;

public final class PhysicalConstants {
    // ————— swerve constants ————— //
    // specs // ! recalculate these
    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(2); // 4.4759614398144
    public static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(3); // 9
    public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI); // 8 * pi
    public static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Seconds).of(2 * Math.PI); // 10 * pi
    
    // kinematics
    public static final Mass ROBOT_WEIGHT = Pounds.of(97.666);
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final Distance TRACK_WIDTH_X = Inches.of(23.625);
    public static final Distance TRACK_WIDTH_Y = Inches.of(27.75);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] { // using the chassisspeeds coordinate plane
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // FL
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0), // FR
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // BL
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0) // BR
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    // gears
    public static final double DRIVE_AFTER_ENCODER_REDUCTION = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_AFTER_ENCODER_REDUCTION = 150.0 / 7.0;
    public static final double COUPLING_RATIO = 50.0 / 14.0;

    // ————— coral constants ————— //
    public static final Angle ELEVATOR_ENCODER_OFFSET = Rotations.of(0.39599609375);
    public static final Angle ELEVATOR_MAX_ROTATIONS = Rotations.of(3.612548828125);
    public static final Angle ELEVATOR_MIN_ROTATIONS = Rotations.of(0);

    // recalc: https://www.reca.lc/arm?armMass=%7B%22s%22%3A4.59%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A11.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    public static final Angle ARM_ENCODER_OFFSET = Rotations.of(-0.260986328125);
    public static final Angle ARM_MAX_ROTATIONS = Rotations.of(0.582763671875); // 0.922119140625 when elevator fully up
    public static final Angle ARM_MIN_ROTATIONS = Rotations.of(0);

    public static class CoralPositions{
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
    }

}