// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 *  daily at home: 
 *   sysid for good choreo constants
 *   write coralIOSim
 *   pid tunable constants
 * 
 * — daily at robotics: 
 * —— driving: 
 * *—— test driving on the ground (rotating too)
 * —— coral: 
 * ——— test position control
 * ——— get cancoder offsets
 * ——— make coral code based on absolute encoders instead of relative
 * 
 * !ask colin why the inputmodulus exists
 * 
 * ! alliance problems, change to blue in drive.java probably
 * 
 *  longer term: 
 *   put coralIO inputs in separate folders
 *   controllers for the year
 *   rotating while translating doesn't work at all in sim
 *   movetoapriltag command(assume you can get an angle and distance from the camera)
 *   maybe leds for telling the coral station person whether the robot can intake
 *   make poseestimator its own class, record robot pose not under realoutputs
 * 
 *  current major goal: 
 *   get 2coral auto working: 
 *    driving (in general) and position control
 *    // setting elevator position
 *    setting arm position
 *    setting wrist position
 *    setting claw position
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}