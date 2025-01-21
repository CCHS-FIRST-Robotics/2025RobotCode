// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.subsystems.algaIO.*;
import frc.robot.utils.AutoRoutineGenerator;
import frc.robot.utils.PoseEstimator;

public class RobotContainer {
    private final CommandXboxController controller1 = new CommandXboxController(Constants.CONTROLLER_PORT_1);

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Elevator elevator;
    private final Coral coral;
    private final Alga alga;

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
                elevator = new Elevator(new ElevatorIOTalonFX(Constants.ELEVATOR_ID_1));
                coral = new Coral(new CoralIOTalonFX(Constants.CORAL_CLAW_ID_1));
                alga = new Alga(new AlgaIOTalonFX(Constants.ALGA_ID_1));
                break;
            case SIM: // ! make everything actually a sim class
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                elevator = new Elevator(new ElevatorIOTalonFX(Constants.ELEVATOR_ID_1));
                coral = new Coral(new CoralIOTalonFX(Constants.CORAL_CLAW_ID_1));
                alga = new Alga(new AlgaIOTalonFX(Constants.ALGA_ID_1));
                break;
            default:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSparkMax(0),
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3)
                );
                elevator = new Elevator(new ElevatorIOTalonFX(Constants.ELEVATOR_ID_1));
                coral = new Coral(new CoralIOTalonFX(Constants.CORAL_CLAW_ID_1));
                alga = new Alga(new AlgaIOTalonFX(Constants.ALGA_ID_1));
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

        autoGenerator = new AutoRoutineGenerator(
            drive, 
            elevator,
            coral 
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

        controller1.b().onTrue(coral.getIntakeCommand());
        controller1.a().onTrue(alga.getIntakeCommand());
    }

    private void configureAutos(){
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoral());

        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand();
    }
}