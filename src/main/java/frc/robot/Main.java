// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * get check wrist and claw position control
 * get coral reef positions
 * 
 * run sysid on drivebase (wait until trough and claw are on)
 *  daily at home: 
 * 
 * ! update coral position struct
 * ! check stuff when changing alliances
 * 
 * ! sysid on coral
 * 
 * ! at some point decide on max allowed velocity
 * 
 * — daily at robotics: 
 * —— coral: 
 * *—— wrist position control
 * *—— sysid
 * —— drive: 
 * *—— choreo autos
 * —— alga: 
 * ——— test drawbridge
 * 
 *  longer term: 
 *   zero the drive modules correctly
 *   figure out how to maintain wheel position
 *   write coralIOSim
 *   pid tunable constants
 *   controllers for the year
 *   fused cancoder instead of remote
 *   see if double gyro works
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}