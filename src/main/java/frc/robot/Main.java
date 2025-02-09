// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 *  daily at home: 
 *   put coralIO inputs in separate folders
 *   sysid for good choreo constants
 *    replace kA in ModuleIO classes
 *   check drive position control
 * 
 * — daily at robotics: 
 * —— drive: 
 * ——— test max speed
 * ——— test sysid
 * ——— update choreo
 * *—— test autos
 * —— coral: 
 * ——— absolute encoders
 * ——— test kG on arm
 * —— alga: 
 * ——— add ir sensors
 * ——— code release mechanism
 * 
 *  longer term: 
 *   write coralIOSim
 *   pid tunable constants
 *   controllers for the year
 *   movetoapriltag command(assume you can get an angle and distance from the camera)
 *   maybe leds for telling the coral station person whether the robot can intake
 *   make poseestimator its own class, record robot pose not under realoutputs
 * 
 *  current major goal: 
 *   get 2coral auto working: 
 *    driving (in general) and position control
 *    setting elevator position
 *    setting arm position
 *    setting wrist position
 *    setting claw position
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}