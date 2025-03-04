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
 * ! write manual autos
 * ! check stuff when changing alliances
 * 
 * ! drive sysid
 * ! get coral encoder offsets
 * ! get coral positions
 * ! see whether trough sensor is ir or switch
 * ! sysid on coral
 * ! see how they sauter the claw limit switch (always on or not)
 * ! turn sysid
 * ! change poseEstimator starting position for autos?
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