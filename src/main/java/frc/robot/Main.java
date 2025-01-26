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
 * go through module classes
 * maybe pid tunable constants
 * 
 * — at robotics daily: 
 * —— change id numbers
 * —— test driving with joysticks
 * —— test autos
 * 
 * longer term: 
 * sysid (or just use recalc)
 * ! rotating while translating doesn't work at all in sim
 * volts, amps, rotations, rotations per second, celcius
 * good choreo constants
 * navx logs in degrees (might be a problem)
 * go through all units and conversions in general
 * understand advantagescope positions of everything and where everything is logged
 * decide on controllers for the year
 * make sure the position setpoint is actually informed by the poseestimator
 * 
 * cosmetic / housekeeping: 
 * move periodic functions to top
 * go through imports at some point, make stuff private final
 * thetaa in drive.java
 * maybe rename arm to pivot, axle, or axis
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}