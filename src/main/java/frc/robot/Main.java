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
 *  daily: 
 *   pid tunable constants
 *   sysid (or just use recalc) for good choreo constants
 *   write elevator and arm cancoder code
 *   write coralIOSim
 * 
 * — at robotics daily: 
 * —— driving: 
 * ——— check where module rotation 0 is
 * *—— check if drivewithjoysticks works
 * ——— test replacing drive.java stop function
 * ——— maybe test 
 * —— arm: 
 * ——— tighten the belt
 * ——— test whether position is set to 0 every deploy
 * ——— test whether the gear ratio config works
 * ——— test going up and down and stopping
 * *—— find the gear ratio
 * ——— test position control
 * 
 *  longer term: 
 *   how to zero the elevator
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