// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * ! make drive backwards to outtake faster
 * ! make elevator faster
 * ! tune arm without motionmagic
 * 
 * ! make gyro relative
 * 
 * 
 * ! alliance flipping will need to exist when vision is a thing
 * ! maybe just add 180 to gyro output when red alliance?
 * 
 * ! test using alliance flipping while the gyro reads 180
 * 
 * ! it would be cool to do the alliance flipping / FOC in drivewithvelocity
 * 
 * ! go through all uses of angles in code and see if it's -pi to pi
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