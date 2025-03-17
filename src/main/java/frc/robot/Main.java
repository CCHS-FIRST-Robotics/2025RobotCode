// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * ! tune arm without motionmagic
 * 
 * to do: 
 * * daily at home: 
 * 
 * * daily at robotics: 
 *    get coral reef positions
 * 
 * * longer term: 
 *    zero the drive modules correctly 
 *    figure out how to maintain wheel position
 *    write coralIOSim
 *    pid tunable constants
 *    fused cancoder instead of remote
 *    see if double gyro works
 *  
 * * game plan: 
 * —  autos: 
 *     set a startingPosition // ! look up where legal starting positions are
 *     to place coral, drive with choreo / position, then drivewithapriltag and place
 *     to intake coral, drive with choreo / position, then drivewithapriltag and wait for ir sensor
 *  
 * /   requirements: 
 *      global pose estimation with apriltags   
 *      drivewithapriltag homing
 *      setting coral position
 *      trough ir sensor
 *  
 * —  teleop: 
 *     to place coral, drive with velocity, then drivewithapriltag and place
 *     to intake coral, drive with velocity, then drivewithapriltag and wait (no ir sensor needed)
 *  
 * /   requirements: 
 *      drivewithapriltag homing
 *      setting coral position
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}