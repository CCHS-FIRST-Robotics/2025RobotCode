// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.apriltag.*;

public final class VirtualConstants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = ROBOT_MODE.REAL;
    
    // controllers
    public static final int XBOX_CONTROLLER_PORT = 1;
    public static final int CORAL_CONTROLLER_PORT = 2;
    public static final double JOYSTICK_DEADZONE = 0.15;
    public static final double LINEAR_SPEED_EXPONENT = 4;
    public static final double ANGULAR_SPEED_EXPONENT = 3;

    // ids
    public static final int ROBORIO_ID = 0;
    public static final int PDH_ID = 1;
    public static final int ELEVATOR_ID = 2;
    public static final int ARM_ID = 3;

    // cancoder ids
    public static final int ELEVATOR_CANCODER_ID = 50;
    public static final int ARM_CANCODER_ID = 51;

    // cameras
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final int NUM_CAMERAS = 2;
    public static final String[] CAMERA_PHOTONVISION_NAMES = {"FrontLeft", "FrontRight"};
    public static final String[] CAMERA_LOGGER_NAMES = {"frontLeft", "frontRight"};
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.4, 0.4, 0.2);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.4);
    public static final double AMBIGUITY_THRESHOLD = 0.2; // "numbers above 0.2 are likely to be ambiguous" - PhotonTarget.getPoseAmbiguity()
    public static final double DISTANCE_SCALAR = 60;
}