
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * organize imports
 * see how to do pathplanner
 * that tells you whether heading PID is necessary
 * add twist support for sim
 */

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}