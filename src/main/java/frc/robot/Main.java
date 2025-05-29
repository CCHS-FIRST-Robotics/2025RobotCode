// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * * daily at home: 
 *    make gyro relative
 *    alliance flipping will need to exist when vision is a thing
 *    maybe just add 180 to gyro output when red alliance?
 *    it would be cool to do the alliance flipping / FOC in drivewithvelocity
 *    go through all uses of angles in code and see if it's -pi to pi
 *    write coralIOSim
 *    tune arm without motionmagic
 * 
 * * daily at robotics: 
 *    check placing coral
 *    check autos
 * 
 * * longer term: 
 *    see if double gyro works
 *    pid tunable constants
 *    fused cancoder instead of remote
 * 
 * * cleanup procedure: 
 *    check imports
 *    all {}
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}