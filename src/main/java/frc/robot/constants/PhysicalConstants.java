package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import java.util.*;

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
    public static LinearVelocity MAX_ALLOWED_LINEAR_SPEED = MetersPerSecond.of(4);
    public static AngularVelocity MAX_ALLOWED_ANGULAR_SPEED = RotationsPerSecond.of(1);
    public static LinearAcceleration MAX_ALLOWED_LINEAR_ACCEL = MetersPerSecondPerSecond.of(20);
    public static AngularAcceleration MAX_ALLOWED_ANGULAR_ACCEL = RotationsPerSecondPerSecond.of(20 / TRACK_CIRCUMFERENCE.in(Meters));

    // ————— poseEstimator constants ————— //

    // jetson
    public static final Angle[] ARDUCAM_PITCHES = {
        Degrees.of(-25), // front
        Degrees.of(-3), // left
        Degrees.of(-3), // right
        Degrees.of(30) // back
    };
    public static final Pose2d JETSON_OFFSET = new Pose2d(
        Inches.of(0.75).in(Meters), 
        Inches.of(0).in(Meters), 
        new Rotation2d(Degrees.of(0))
    );
    public static final Pose2d USB_CAMERA_1_OFFSET = new Pose2d(
        Meters.of(0.1).in(Meters), 
        Meters.of(0.27).in(Meters), 
        new Rotation2d(Degrees.of(-29.3577535419))
    );

    // photonvision
    public static final int NUM_CAMERAS = 4;
    public static final String[] cameraNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    public static final Transform3d FRONT_LEFT_OFFSET = new Transform3d(
        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-5), Units.degreesToRadians(45))
    );
    public static final Transform3d FRONT_RIGHT_OFFSET = new Transform3d(
        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-45))
    );
    public static final Transform3d BACK_LEFT_OFFSET = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(5.5), Units.degreesToRadians(-20), Units.degreesToRadians(45))
    );
    public static final Transform3d BackRightCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(5.5), Units.degreesToRadians(-20), Units.degreesToRadians(-45))
    );
    public static final Transform3d[] cameraTransforms = {
        FRONT_LEFT_OFFSET,
        FRONT_RIGHT_OFFSET,
        BACK_LEFT_OFFSET,
        BackRightCamToCenter
    };

    // ! look later, also check the docs
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 2);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.4);
    public static final int DISTANCE_WEIGHT = 60; // higher means if tags are futher away they make a less difference on stddevs  30 is good starting point
    public static final int TOO_FAR_AWAY = 6; // meters at which tag distances to pose are thrown out

    public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final HashMap<Integer, Pose2d> APRILTAG_LOCATIONS = new HashMap<Integer, Pose2d>();
    static {
        APRILTAG_LOCATIONS.put(1, new Pose2d(16.697198, 0.65532, new Rotation2d(Degrees.of(126))));
        APRILTAG_LOCATIONS.put(2, new Pose2d(16.697198, 7.39648, new Rotation2d(Degrees.of(234))));
        APRILTAG_LOCATIONS.put(3, new Pose2d(11.56081, 8.05561, new Rotation2d(Degrees.of(270))));
        APRILTAG_LOCATIONS.put(4, new Pose2d(9.27608, 6.137656, new Rotation2d(Degrees.of(0))));
        APRILTAG_LOCATIONS.put(5, new Pose2d(9.27608, 1.914906, new Rotation2d(Degrees.of(0))));
        APRILTAG_LOCATIONS.put(6, new Pose2d(13.474446, 3.306318, new Rotation2d(Degrees.of(300))));
        APRILTAG_LOCATIONS.put(7, new Pose2d(13.890498, 4.0259, new Rotation2d(Degrees.of(0))));
        APRILTAG_LOCATIONS.put(8, new Pose2d(13.474446, 4.745482, new Rotation2d(Degrees.of(60))));
        APRILTAG_LOCATIONS.put(9, new Pose2d(12.643358, 4.745482, new Rotation2d(Degrees.of(120))));
        APRILTAG_LOCATIONS.put(10, new Pose2d(12.227306, 4.0259, new Rotation2d(Degrees.of(180))));
        APRILTAG_LOCATIONS.put(11, new Pose2d(12.643358, 3.306318, new Rotation2d(Degrees.of(240))));
        APRILTAG_LOCATIONS.put(12, new Pose2d(0.851154, 0.65532, new Rotation2d(Degrees.of(54))));
        APRILTAG_LOCATIONS.put(13, new Pose2d(0.851154, 7.39648, new Rotation2d(Degrees.of(306))));
        APRILTAG_LOCATIONS.put(14, new Pose2d(8.272272, 6.137656, new Rotation2d(Degrees.of(180))));
        APRILTAG_LOCATIONS.put(15, new Pose2d(8.272272, 1.914906, new Rotation2d(Degrees.of(180))));
        APRILTAG_LOCATIONS.put(16, new Pose2d(5.987542, -0.00381, new Rotation2d(Degrees.of(90))));
        APRILTAG_LOCATIONS.put(17, new Pose2d(4.073906, 3.306318, new Rotation2d(Degrees.of(240))));
        APRILTAG_LOCATIONS.put(18, new Pose2d(3.6576, 4.0259, new Rotation2d(Degrees.of(180))));
        APRILTAG_LOCATIONS.put(19, new Pose2d(4.073906, 4.745482, new Rotation2d(Degrees.of(120))));
        APRILTAG_LOCATIONS.put(20, new Pose2d(4.90474, 4.745482, new Rotation2d(Degrees.of(60))));
        APRILTAG_LOCATIONS.put(21, new Pose2d(5.321046, 4.0259, new Rotation2d(Degrees.of(0))));
        APRILTAG_LOCATIONS.put(22, new Pose2d(4.90474, 3.306318, new Rotation2d(Degrees.of(300))));
    }

    public static final class DrivePositions {
        public static final Translation2d INTAKE = new Translation2d(0, 0); // ! no idea how this would work
        public static final Translation2d L1 = new Translation2d(0, 0);
        public static final Translation2d L2 = new Translation2d(0, 0); // ! 
        public static final Translation2d L3 = new Translation2d(0, 0); // ! 
        public static final Translation2d L4 = new Translation2d(0.63, 0);

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
        public static final Angle[] INTAKE_PREP = {Rotations.of(1), Rotations.of(-0.2191816406)};
        public static final Angle[] INTAKE_RUN = {Rotations.of(0.195556640625), Rotations.of(-0.2131816406)};
        public static final Angle[] L1 = {Rotations.of(0), Rotations.of(0)};
        public static final Angle[] L2 = {Rotations.of(0), Rotations.of(0.014892578125)};
        public static final Angle[] L3 = {Rotations.of(0.263427734375), Rotations.of(0.0986328125)};
        public static final Angle[] L4 = {Rotations.of(3.6831054687499996), Rotations.of(0.13830078125)};
    }
}