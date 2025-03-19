package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.*;
import java.util.*;

public final class PhysicalConstants {

    // ————— drive constants ————— //

    // kinematics
    public static final Mass ROBOT_WEIGHT = Pounds.of(99.2); // battery is 13.324lbs extra
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);
    public static final Distance TRACK_WIDTH_X = Inches.of(22.5);
    public static final Distance TRACK_WIDTH_Y = Inches.of(26.5);
    public static final Distance TRACK_RADIUS = Inches.of(Math.hypot(TRACK_WIDTH_X.in(Inches) / 2.0, TRACK_WIDTH_Y.in(Inches) / 2.0));
    public static final Distance TRACK_CIRCUMFERENCE = TRACK_RADIUS.times(2 * Math.PI);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] { // using the chassisspeeds coordinate plane
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // FL
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0), // FR
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // BL
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0) // BR
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    // gears
    public static final double DRIVE_AFTER_ENCODER_REDUCTION = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.7460317460
    public static final double TURN_AFTER_ENCODER_REDUCTION = 150.0 / 7.0; // 21.4285714286
    public static final double COUPLING_RATIO = 50.0 / 14.0;

    // specs
    public static final LinearVelocity MAX_POSSIBLE_LINEAR_SPEED = MetersPerSecond.of(
        MotorSpecifications.NEO_MAX_VELOCITY.in(RotationsPerSecond) // rotations per second of the motor
        / DRIVE_AFTER_ENCODER_REDUCTION // rotations per second of the wheel
        * WHEEL_CIRCUMFERENCE.in(Meters) // meters per second of the robot
    ); // = 4.6368310899
    public static final AngularVelocity MAX_POSSIBLE_ANGULAR_SPEED = RotationsPerSecond.of(
        MAX_POSSIBLE_LINEAR_SPEED.in(MetersPerSecond) // meters per second of the robot
        / TRACK_CIRCUMFERENCE.in(Meters) // rotations per second of the robot
    ); // = 1.5944299280
    public static final LinearVelocity MAX_ALLOWED_LINEAR_SPEED = MetersPerSecond.of(4);
    public static final AngularVelocity MAX_ALLOWED_ANGULAR_SPEED = RotationsPerSecond.of(1);
    public static final LinearAcceleration MAX_ALLOWED_LINEAR_ACCEL = MetersPerSecondPerSecond.of(20);
    public static final AngularAcceleration MAX_ALLOWED_ANGULAR_ACCEL = RotationsPerSecond.per(Seconds).of(20 / TRACK_CIRCUMFERENCE.in(Meters));

    // ————— poseEstimator constants ————— //

    public static final Transform3d JETSON_OFFSET = new Transform3d(
        Inches.of(-2).in(Meters), 
        Inches.of(0).in(Meters),
        Inches.of(34).in(Meters),
        new Rotation3d()
    );

    public static final HashMap<Integer, Pose3d> APRILTAG_LOCATIONS = new HashMap<Integer, Pose3d>();
    static {
        APRILTAG_LOCATIONS.put( 1, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 2, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 3, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 4, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 5, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 6, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 7, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 8, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put( 9, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(10, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(11, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(12, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(13, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(14, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(15, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(16, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(17, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(18, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(19, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(20, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(21, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        APRILTAG_LOCATIONS.put(22, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        // APRILTAG_LOCATIONS.put(1, new Translation3d(16.697, 0.655, 1.486));
        // APRILTAG_LOCATIONS.put(2, new Translation3d(16.697, 7.396, 1.486));
        // APRILTAG_LOCATIONS.put(3, new Translation3d(11.561, 8.056, 1.302));
        // APRILTAG_LOCATIONS.put(4, new Translation3d(9.276, 6.138, 1.868));
        // APRILTAG_LOCATIONS.put(5, new Translation3d(9.276, 1.915, 1.868));
        // APRILTAG_LOCATIONS.put(6, new Translation3d(13.474, 3.306, 0.308));
        // APRILTAG_LOCATIONS.put(7, new Translation3d(13.890, 4.026, 0.308));
        // APRILTAG_LOCATIONS.put(8, new Translation3d(13.474, 4.745, 0.308));
        // APRILTAG_LOCATIONS.put(9, new Translation3d(12.643, 4.745, 0.308));
        // APRILTAG_LOCATIONS.put(10, new Translation3d(12.227, 4.026, 0.308));
        // APRILTAG_LOCATIONS.put(11, new Translation3d(12.643, 3.306, 0.308));
        // APRILTAG_LOCATIONS.put(12, new Translation3d(0.851, 0.655, 1.486));
        // APRILTAG_LOCATIONS.put(13, new Translation3d(0.851, 7.396, 1.486));
        // APRILTAG_LOCATIONS.put(14, new Translation3d(8.272, 6.138, 1.868));
        // APRILTAG_LOCATIONS.put(15, new Translation3d(8.272, 1.915, 1.868));
        // APRILTAG_LOCATIONS.put(16, new Translation3d(5.988, -0.004, 1.302));
        // APRILTAG_LOCATIONS.put(17, new Translation3d(4.074, 3.306, 0.308));
        // APRILTAG_LOCATIONS.put(18, new Translation3d(3.658, 4.026, 0.308));
        // APRILTAG_LOCATIONS.put(19, new Translation3d(4.074, 4.745, 0.308));
        // APRILTAG_LOCATIONS.put(20, new Translation3d(4.905, 4.745, 0.308));
        // APRILTAG_LOCATIONS.put(21, new Translation3d(5.321, 4.026, 0.308));
        // APRILTAG_LOCATIONS.put(22, new Translation3d(4.905, 3.306, 0.308));
    }

    // ————— coral constants ————— //

    public static final Angle ELEVATOR_ENCODER_OFFSET = Rotations.of(0.466796875);
    public static final Angle ELEVATOR_MAX_ROTATIONS = Rotations.of(3.6831054687499996);
    public static final Angle ELEVATOR_MIN_ROTATIONS = Rotations.of(0);
    public static final double ELEVATOR_GEAR_REDUCTION = 100;

    public static final Angle ARM_ENCODER_OFFSET = Rotations.of(0.405029296875);
    public static final Angle ARM_MAX_ROTATIONS = Rotations.of(0.13830078125);
    public static final Angle ARM_MIN_ROTATIONS = Rotations.of(0);
    public static final double ARM_GEAR_REDUCTION = 100;

    public static class CoralPositions { // elevator rotations, arm rotations
        public static final Angle[] INTAKE_PREP = {Rotations.of(1), Rotations.of(-0.2191816406)}; // -0.211
        public static final Angle[] INTAKE_RUN = {Rotations.of(0.195556640625), Rotations.of(-0.2131816406)};
        public static final Angle[] L1 = {Rotations.of(0), Rotations.of(0)};
        public static final Angle[] L2 = {Rotations.of(0), Rotations.of(0.014892578125)};
        public static final Angle[] L3 = {Rotations.of(1), Rotations.of(0.075556640625)};
        public static final Angle[] L4 = {Rotations.of(3.6831054687499996), Rotations.of(0.13830078125)};
    }
}