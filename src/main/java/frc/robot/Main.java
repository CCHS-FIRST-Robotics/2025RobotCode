// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 *  daily at home: 
 *   check drive position control
 *   sysid
 * 
 * — daily at robotics: 
 * —— coral: 
 * ——— wrist
 * —— drive: 
 * ——— test max speed
 * ——— test position control
 * ———— drivePosition logging code was changed
 * *—— test autos
 * —— alga: 
 * ——— code release mechanism
 * 
 *  longer term: 
 *   zero the drive modules correctly
 *   write coralIOSim
 *   pid tunable constants
 *   controllers for the year
 *   make poseestimator its own class, record robot pose not under realoutputs
 *   fused cancoder instead of remote
 *   ir sensors
 *   maybe add kV values from datasheets to motorspecs (probably will be overridden by sysid)
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}