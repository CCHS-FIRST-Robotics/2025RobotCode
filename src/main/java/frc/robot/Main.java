// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 *  daily at home: 
 *   wrist code
 *   sysid everything
 *   check drive position control
 * 
 * — daily at robotics: 
 * —— coral: 
 * ——— wrist code
 * ——— limit switches for elevator
 * —— drive: 
 * ——— test max speed
 * ——— test position control
 * ———— drivePosition logging code was changed
 * *—— test autos
 * —— alga: 
 * ——— add ir sensors
 * ——— code release mechanism
 * 
 *  longer term: 
 *   zero the drive modules correctly
 *   write coralIOSim
 *   pid tunable constants
 *   controllers for the year
 *   make poseestimator its own class, record robot pose not under realoutputs
 *   fused cancoder instead of remote
 * 
 *  current major goal: 
 *   get 2coral auto working: 
 *    driving (in general) and position control
 *    // setting elevator position
 *    // setting arm position
 *    setting wrist position
 *    setting claw position
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}