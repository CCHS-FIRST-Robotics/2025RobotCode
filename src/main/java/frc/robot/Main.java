// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * * things to keep in mind: 
 *    maybe test using the coupling thing again
 *    drive smart current limits
 *    drivewithapriltag velocity rampdown
 *    knock out algae
 * 
 * * daily at home: 
 *    write coralIOSim
 *    figure out a cleanup procedure:
 *     imports
 *     all {}
 * 
 * * daily at robotics: 
 * !  mechanical / electrical: 
 * !   take apart the arm gearbox
 * !   wire trough sensor
 * !  me: 
 * !   write the pose3d code for apriltags (easy)
 * !   drivewithapriltag needs to know which L they want to go to
 * !    maybe make a constant in robotcontainer that tracks what the last L to go was and then pass it in as an argument for drivewithapriltags
 * !   show in elastic which apriltags it can see
 * !   check apriltag hashmap
 * 
 * 
 * 
 * 
 * ! we need an easy way to convert from autos on blue team to red team
 * 
 * ! figure out what to do about the elevator cancoder (probably just adjust all elevator values by -1)
 * 
 * * longer term: 
 *    zero the drive modules correctly 
 *    pid tunable constants
 *    fused cancoder instead of remote
 *    see if double gyro works
 *    tune arm without motionmagic
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