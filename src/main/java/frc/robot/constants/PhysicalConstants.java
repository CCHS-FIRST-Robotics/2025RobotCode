package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.*;

public final class PhysicalConstants {

    // ————— drive constants ————— //

    // kinematics
    public static final Mass ROBOT_WEIGHT = Pounds.of(98.568); // battery is 13.324lbs extra
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
    public static LinearVelocity MAX_ALLOWED_LINEAR_SPEED = MetersPerSecond.of(1);
    public static AngularVelocity MAX_ALLOWED_ANGULAR_SPEED = RotationsPerSecond.of(0.5);
    public static LinearAcceleration MAX_ALLOWED_LINEAR_ACCEL = MetersPerSecondPerSecond.of(20);
    public static AngularAcceleration MAX_ALLOWED_ANGULAR_ACCEL = RotationsPerSecondPerSecond.of(20 / TRACK_CIRCUMFERENCE.in(Meters));

    // ————— poseEstimator constants ————— //

    // cameras
    public static final Transform3d FRONT_LEFT_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-5), Units.degreesToRadians(45))
    );
    public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-45))
    );
    public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(5.5), Units.degreesToRadians(-20), Units.degreesToRadians(45))
    );
    public static final Transform3d BACK_RIGHT_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(5.5), Units.degreesToRadians(-20), Units.degreesToRadians(-45))
    );

    // public static final Transform3d FRONT_LEFT_TRANSFORM = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(-14), Units.inchesToMeters(0)),
    //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-30))
    // );
    // public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(14), Units.inchesToMeters(0)),
    //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-30))
    // );
    // public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(14), Units.inchesToMeters(0)),
    //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
    // );
    // public static final Transform3d BACK_RIGHT_TRANSFORM = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(-14), Units.inchesToMeters(0)),
    //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))
    // );

    public static final Transform3d[] CAMERA_TRANSFORMS = {
        FRONT_LEFT_TRANSFORM,
        FRONT_RIGHT_TRANSFORM,
        BACK_LEFT_TRANSFORM,
        BACK_RIGHT_TRANSFORM
    };

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
        public static final Angle[] INTAKE_PREP = {Rotations.of(1), Rotations.of(-0.2191816406)};
        public static final Angle[] INTAKE_RUN = {Rotations.of(0.195556640625), Rotations.of(-0.2131816406)};
        public static final Angle[] L1 = {Rotations.of(0), Rotations.of(0)};
        public static final Angle[] L2 = {Rotations.of(0), Rotations.of(0.014892578125)};
        public static final Angle[] L3 = {Rotations.of(0.263427734375), Rotations.of(0.0986328125)};
        public static final Angle[] L4 = {Rotations.of(3.6831054687499996), Rotations.of(0.13830078125)};
    }
}