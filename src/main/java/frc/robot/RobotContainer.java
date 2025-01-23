// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.*;
import frc.robot.utils.AutoRoutineGenerator;

public class RobotContainer {
    private final CommandXboxController controller1 = new CommandXboxController(Constants.CONTROLLER_PORT_1);

    private final Drive drive;

    private final AutoRoutineGenerator autoGenerator;
    private final AutoChooser autoChooser;

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

        autoGenerator = new AutoRoutineGenerator(
            drive, 
            null,
            null 
        );
        autoChooser = new AutoChooser();
        
        configureButtonBindings();
        configureAutos();
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

        // controller1.b().onTrue(coral.getIntakeCommand());
        // controller1.a().onTrue(alga.getIntakeCommand());
    }

    private void configureAutos(){
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoral());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand();
    }
}