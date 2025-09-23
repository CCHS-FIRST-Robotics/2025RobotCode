// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * ! try resetting pose to 0 and 180 degrees, see if the gyro rotation is really taken care of without even rezeroing the gyro. if so, I don't need to worry about relative gyro whatever
 * 
 * ! whenever we use robot rotation, confine to between 0 and 1 rotation using inputmodulus
 * ! check if the poseestimate angle is confined between two values, and if not, if setposition does what I think it should. 
 * !    remember to commenting out isFinished in drivewithposition.java
 * 
 * ! also testing with multiple apriltags to see if multitagpnp is working
 * 
 * 
 * 
 * clean up imports
 * 
 * goal for preseason: 
 * get the oneCoralL4 auto to work when on both red and blue alliance
 * 
 * * tuning
 *    tune arm without motionmagic
 *    figure out sysid
 * 
 * * longer term: 
 *    pid tunable constants
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}