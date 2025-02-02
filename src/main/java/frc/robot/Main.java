// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * ! how did the arm on 2024 robot go to posiion 0 on startup
 * 
 * 
 *  daily: 
 *   fix real classes
 *   fix hickey's code
 *   pid tunable constants
 * 
 * — at robotics daily: 
 * ——— test the arm
 * ——— check where module rotation 0 is
 * ——— test replacing drive.java stop function
 * —— add coral code to autos
 * 
 *  longer term: 
 *   sysid (or just use recalc) for good choreo constants
 *   ! rotating while translating doesn't work at all in sim
 *   go through all unit conversions
 *   controllers for the year
 *   movetoapriltag command(assume you can get an angle and distance from the camera)
 *   maybe leds for telling the coral station person whether the robot can intake
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}