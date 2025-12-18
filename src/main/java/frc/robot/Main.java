// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * * tuning
 *    tune arm without motionmagic
 * 
 * * longer term: 
 *    pid tunable constants
 *       https://discord.com/channels/176186766946992128/368993897495527424/1438631090578259968
 *    figure out sysid
 *    rotating around a swerve module
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}