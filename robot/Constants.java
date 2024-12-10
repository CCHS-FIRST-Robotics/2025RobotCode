// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum Mode {REAL, SIM, REPLAY}
    public static final Mode MODE = Mode.REAL;
    
    public static final int CONTROLLER_PORT = 0;
    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double JOYSTICK_EXPONENT = 2;

    public static final int TALONSRX_ID_1 = 23;
    public static final int TALONSRX_ID_2 = 2;
    public static final int TALONSRX_ID_3 = 10;
    public static final int TALONSRX_ID_4 = 11;

    public static final Measure<Distance> TRACK_WIDTH = Centimeters.of(58);
    public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3); 
    public static final Measure<Distance> WHEEL_CIRCUMFERENCE = Inches.of(2 * Math.PI * WHEEL_RADIUS.in(Inches));
    public static final double GEAR_RATIO = 12.755; // for every GEAR_RATIO rotations of the input, the output rotates once
    // public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(3.334);
    public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(1);
}