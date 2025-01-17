// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.utils.*;

public class RobotContainer {
    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final CommandXboxController controller1 = new CommandXboxController(Constants.CONTROLLER_PORT_1);

    // ! auto chooser

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSparkMax(0),
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3)
                );
                break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                break;
            default:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSparkMax(0),
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3)
                );
                break;
        }

        // ! wow this looks like it should be automized
        // change pose here for autos!!!
        poseEstimator = new PoseEstimator(
            HardwareConstants.KINEMATICS,
            new Rotation2d(),
            drive.getModulePositions(),
            new Pose2d(0, 0, new Rotation2d())
        );

        drive.setPoseEstimator(poseEstimator);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> controller1.getLeftX(), 
                () -> -controller1.getLeftY(), // xboxcontroller is flipped
                () -> controller1.getRightX()
            )
        );
    }

    public Command getAutonomousCommand() {
        return new AutoCommandSequenceBuilder(
            Constants.AutoConstants.twoStraight, // specifies the auto to run
            drive
        ).getAutoCommandSequence();
    }
}