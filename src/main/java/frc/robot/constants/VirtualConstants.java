// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VirtualConstants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = ROBOT_MODE.REAL;
    
    // controller
    public static final int CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;
    public static final int CONTROLLER_PORT_3 = 2;
    public static final double JOYSTICK_DEADZONE = 0.05;
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

  
    public static final int TROUGH_SWITCH_PORT = 0;
    public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(.8, .8, 2);
    public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.2, 0.2, .4);
    public static final Matrix<N3, N1> DriveStdDevs = VecBuilder.fill(0.9, 0.9, .1);



    public static final int DistanceWeight = 60; // higher means if tags are futher away they make a less difference on stddevs  30 is good starting point
    public static final int ToFarAway = 6; // Meters at which tag distances to pose are thrown out


}