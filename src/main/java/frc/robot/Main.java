// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * current goal is literally just be able to score L4 with apriltag
 * ! maybe ask colin how come you can't drivewithjoysticks out of a drivewithapriltag
 * ! gabe's controllers with just pressing the left or right trigger and it goes to whichever apriltag it sees
 * 
 * to do: 
 * * things to keep in mind: 
 * 
 * * daily at home: 
 *    write coralIOSim
 *    figure out a cleanup procedure:
 *     imports
 *     all {}
 * 
 * * daily at robotics: 
 * ! we need an easy way to convert from autos on blue team to red team
 * 
 * * longer term: 
 *    knock out algae
 *    maybe test using the coupling thing again
 *    zero the drive modules correctly 
 *    pid tunable constants
 *    fused cancoder instead of remote
 *    see if double gyro works
 *    tune arm without motionmagic
 *    drivewithaprixltag velocity rampdown
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