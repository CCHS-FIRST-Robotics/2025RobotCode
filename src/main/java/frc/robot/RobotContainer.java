// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.subsystems.algaIO.*;
import frc.robot.utils.AutoRoutineGenerator;
import frc.robot.constants.*;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(VirtualConstants.CONTROLLER_PORT);

    private final Drive drive;
    private final Coral coral;
    private final Alga alga;

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
                        VirtualConstants.ELEVATOR_CANCODER_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.ARM_CANCODER_ID, 
                        VirtualConstants.WRIST_ID, 
                        VirtualConstants.WRIST_CANCODER_ID, 
                        VirtualConstants.CLAW_ID
                    )
                );

                alga = new Alga(new AlgaIOTalonFX(VirtualConstants.ALGA_ID_1));
                break;
            case SIM:
                drive = new Drive(
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new GyroIO() {}
                );

                coral = new Coral(new CoralIOSim());

                alga = new Alga(new AlgaIOSim());
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
                        VirtualConstants.ELEVATOR_CANCODER_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.ARM_CANCODER_ID, 
                        VirtualConstants.WRIST_ID, 
                        VirtualConstants.WRIST_CANCODER_ID, 
                        VirtualConstants.CLAW_ID
                    )
                );

                alga = new Alga(new AlgaIOTalonFX(VirtualConstants.ALGA_ID_1));
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
        // ————— driving ————— //
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> -controller.getLeftY(), // xboxcontroller is flipped
                () -> controller.getLeftX(), 
                () -> controller.getRightX()
            )
        );

        // ————— elevator ————— //
        // controller.y().onTrue(coral.getSetElevatorCommand(PhysicalConstants.ELEVATOR_MAX_ROTATIONS));
        // controller.a().onTrue(coral.getSetElevatorCommand(PhysicalConstants.ELEVATOR_MIN_ROTATIONS));

        // ————— arm ————— //
        // controller.x().onTrue(coral.getSetArmCommand(PhysicalConstants.ARM_MAX_ROTATIONS);
        // controller.b().onTrue(coral.getSetArmCommand(PhysicalConstants.ARM_MIN_ROTATIONS));

        controller.x().onTrue(coral.getSetWristVoltageCommand(Volts.of(1)));
        controller.b().onTrue(coral.getSetWristVoltageCommand(Volts.of(-1)));
        controller.y().onTrue(coral.getSetWristVoltageCommand(Volts.of(0)));

        // ————— alga ————— //
        // controller.x().onTrue(alga.getIntakeCommand());
        // controller.b().onTrue(alga.getOutputCommand());
    }

    private void configureAutos(){
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoral());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand();
    }
}