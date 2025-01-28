// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * daily: 
 * write coral commands and hashmap
 * maybe pid tunable constants
 * 
 * — at robotics daily: 
 * —— change CAN id numbers (CAN wiring is so fun)
 * —— test driving with joysticks (remember to change the robot mode)
 * ——— check where module rotation 0 is
 * ——— test replacing drive.java stop function
 * —— test autos
 * 
 * longer term: 
 * sysid (or just use recalc) for good choreo constants
 * ! rotating while translating doesn't work at all in sim
 * go through all unit conversions
 * controllers for the year
 * movetoapriltag command(assume you can get an angle and distance from the camera)
 * 
 * cosmetic / housekeeping: 
 * move periodic functions to top
 * go through imports, make stuff private final
 * heading in drive.java
 * maybe rename arm to pivot, axle, or axis
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}