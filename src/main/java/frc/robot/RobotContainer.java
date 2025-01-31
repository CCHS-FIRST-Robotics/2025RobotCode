// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.utils.AutoRoutineGenerator;
import frc.robot.constants.VirtualConstants;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(VirtualConstants.CONTROLLER_PORT_1);

    private final Drive drive;
    private final Coral coral;

    private final AutoRoutineGenerator autoGenerator;
    private final AutoChooser autoChooser;

    public RobotContainer() {
        switch (VirtualConstants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3),
                    new ModuleIOSparkMax(4), 
                    new GyroIONavX()
                );

                coral = new Coral(
                    new CoralIOReal(
                        VirtualConstants.ELEVATOR_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.WRIST_ID, 
                        VirtualConstants.CLAW_ID
                    )
                );
                break;
            case SIM:
                drive = new Drive(
                    new ModuleIOSim(1),
                    new ModuleIOSim(2),
                    new ModuleIOSim(3),
                    new ModuleIOSim(4),
                    new GyroIO() {}
                );

                coral = new Coral(new CoralIO() {});
                break;
            default:
                drive = new Drive(
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3),
                    new ModuleIOSparkMax(4),
                    new GyroIONavX()
                );

                coral = new Coral(
                    new CoralIOReal(
                        VirtualConstants.ELEVATOR_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.WRIST_ID, 
                        VirtualConstants.CLAW_ID
                    )
                );
                break;
        }

        autoGenerator = new AutoRoutineGenerator(
            drive
        );
        autoChooser = new AutoChooser();
        
        configureButtonBindings();
        configureAutos();
    }



    private void configureButtonBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> controller.getLeftX(), 
                () -> -controller.getLeftY(), // xboxcontroller is flipped
                () -> controller.getRightX()
            )
        );

        controller.x().onTrue(coral.getArmOnCommand());
        controller.y().onTrue(coral.getElevatorUpCommand());
        controller.a().onTrue(coral.getElevatorDownCommand());
        controller.b().onTrue(coral.getStopElevatorCommand());
    }

    private void configureAutos(){
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoral());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand();
    }
}