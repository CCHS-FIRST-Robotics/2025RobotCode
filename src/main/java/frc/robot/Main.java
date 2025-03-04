// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * ! check stuff when changing alliances
 * ! check alga current limits
 * ! alga limit switches
 * ! figure out poseEstimator and camera
 * 
 * test the alga drawbridge (just voltage control, then ask hickey to mount the limit switches)
 * test the arm and wrist position control
 * run sysid on the whole coral
 * maybe get coral reef positions
 * 
 * maybe run sysid on drivebase (wait until trough and claw are on)
 * maybe test if camera pose estimate works
 * maybe test if pose estimator updates with turning while robot is real
 * 
 * ask jonathan if he can figure out how to configure radios to 6ghz (distract sleder with this)
 * 
 * ! write manual position autos
 * 
 *  daily at home: 
 *   organize imports
 *   private finals
 *   write manual autos
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