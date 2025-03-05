// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * test the alga drawbridge
 * test the arm and wrist position control
 * run sysid on the whole coral
 * get coral reef positions
 * 
 * run sysid on drivebase (wait until trough and claw are on)

 *  daily at home: 
 * ! check stuff when changing alliances
 * 
 * ! check if colin's code fixed the problem with the timing stuff
 * 
 * ! check the discrepancy between fieldposition and odometryposition irl (hopefully odom is more accurate)
 * 
 * ! drive sysid
 * ! get coral encoder offsets
 * ! get coral positions
 * ! see whether trough sensor is ir or switch
 * ! sysid on coral
 * ! see how they sauter the claw limit switch (always on or not)
 * ! turn sysid
 * 
 * 
 * ! look up what counts as a legal auto (how coral starts on robot etc.)
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