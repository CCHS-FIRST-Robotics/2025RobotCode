// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * Talon for elevator
 * one kraken for rotating the arm (I mean like realistically it'll be two)
 * neos for coral claw and wrist
 * 
 * 
 * write coral class
 * write sim classes
 * write drive functions
 * 
 * decide on controllers for the year
 * move periodic functions to top
 * go through imports at some point, make stuff private final
 * change hardwareconstants
 * what the fuck is sysid
 * 
 * maybe combine coral subsystem with elevator? they're basically one subsystem 
 * (coral needs to be able to knock algae off)
 */

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}