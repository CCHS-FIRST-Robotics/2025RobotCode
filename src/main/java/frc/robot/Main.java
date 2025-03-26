// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 //! todo
    test + buy stuff
    try to test at home wtih single camera
    incoperate trust in each pose/tag need to be able to test for that probaly
    tune std dev for drive and apriltags


 
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}