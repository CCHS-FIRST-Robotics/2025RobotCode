// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class VirtualConstants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = ROBOT_MODE.SIM;
    
    // controllers
    public static final int CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;
    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double LINEAR_SPEED_EXPONENT = 2;
    public static final double ANGULAR_SPEED_EXPONENT = 2;

    // ids
    public static final int ELEVATOR_ID = 0;
    public static final int ARM_ID = 0;
    public static final int WRIST_ID = 0;
    public static final int CLAW_ID = 0;
    public static final int TROUGH_SENSOR_PORT = 0;

    public static final int ALGA_ID_1 = 0;
    public static final int ALGA_SENSOR_PORT = 0;
}