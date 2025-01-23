// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * daily: 
 * 
 * 
 * — at robotics daily: 
 * —— flash the rio for 2025
 * —— change hardwareconstants and id numbers
 * —— test driving with joysticks
 * —— update choreo with new hardwareconstants, regenerate auto
 * —— test autos
 * 
 * longer term: 
 * go through module classes
 * navx logs in degrees (might be a problem)
 * go through all units and conversions in general
 * understand advantagescope positions of everything and where everything is logged
 * decide on controllers for the year
 * 
 * cosmetic / housekeeping: 
 * move periodic functions to top
 * go through imports at some point, make stuff private final
 * thetaa in drive.java
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}