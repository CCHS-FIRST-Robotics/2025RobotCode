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
    
    // controller
    public static final int XBOX_CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;
    public static final int CORAL_CONTROLLER_PORT_3 = 2;
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
    public static final int NUM_CAMERAS = 4;
    public static final String[] CAMERA_PHOTONVISION_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    public static final String[] CAMERA_LOGGER_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    // ! look later, also check the docs
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 2);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.4);
    public static final int DISTANCE_WEIGHT = 60; // higher means if tags are futher away they aren't accounted for as much
    public static final int TOO_FAR_AWAY = 6; // meters at which tag estimates are thrown out

    public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
}