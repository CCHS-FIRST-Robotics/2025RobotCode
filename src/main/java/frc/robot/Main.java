// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
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