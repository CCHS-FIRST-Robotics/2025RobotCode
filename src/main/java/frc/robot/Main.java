// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * advantagescope units
 * volts, amps, celcius, rotations, rotationspersecond
 * exceptions are in comments
 * 
 * 
 * confine all numbers to constants, and move them there. subsystem files should not have any numbers
 * 
 * ! is it okay for the robot to assume you're at the 0 angle when you start
 * 
 * use record for structs
 * 
 * 
 * 
 * clean up imports
 * fix spelling of stdevs
 * 
 * goal for preseason: 
 * get the oneCoralL4 auto to work when on both red and blue alliance
 * 
 * * figure out the aliance flipping/gyro whatever
 *    alliance flipping will need to exist when vision is a thing
 *    maybe just add 180 to gyro output when red alliance?
 *    it would be cool to do the alliance flipping / FOC in drivewithvelocity
 *    go through all uses of angles in code and see if it's -pi to pi
 *    make gyro relative
 * 
 * * tuning
 *    tune arm without motionmagic
 *    figure out sysid
 * 
 * * daily at home: 
 * 
 * * daily at robotics: 
 * 
 * * longer term: 
 *    pid tunable constants
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}