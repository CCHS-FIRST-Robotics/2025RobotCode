// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * re-get robot measurements
 * 
 * figure out sysid
 * 
 * goal for preseason: 
 * get the oneCoralL4 auto to work when on both red and blue alliance
 * 
 * 
 * rotating around a swerve module
 * 
 * 
 * clean up imports
 * 
 * * tuning
 *    tune arm without motionmagic
 * 
 * * longer term: 
 *    pid tunable constants
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}