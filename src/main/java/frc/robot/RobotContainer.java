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
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.subsystems.algaIO.*;
import frc.robot.utils.*;
import frc.robot.constants.*;

public class RobotContainer {
    private final CommandXboxController controller1 = new CommandXboxController(VirtualConstants.CONTROLLER_PORT_1);
    private final CommandXboxController controller2 = new CommandXboxController(VirtualConstants.CONTROLLER_PORT_2);

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Coral coral;
    private final Alga alga;

    private final AutoRoutineGenerator autoGenerator;
    private final AutoChooser autoChooser;

    public RobotContainer() {
        switch (VirtualConstants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                    new ModuleIOReal(1),
                    new ModuleIOReal(2),
                    new ModuleIOReal(3),
                    new ModuleIOReal(4)
                );

                poseEstimator = new PoseEstimator(
                    new GyroIOReal(),
                    new CameraIOReal(),
                    drive
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
                    ), 
                    VirtualConstants.TROUGH_SWITCH_PORT
                );

                alga = new Alga(
                    new AlgaIOReal(
                        VirtualConstants.ALGA_ID_1,
                        VirtualConstants.ALGA_ID_2
                    ), 
                    VirtualConstants.ALGA_UP_SWITCH_PORT,
                    VirtualConstants.ALGA_DOWN_SWITCH_PORT
                );
                break;
            case SIM:
                drive = new Drive(
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );

                poseEstimator = new PoseEstimator(
                    new GyroIO() {},
                    new CameraIO() {},
                    drive
                );

                coral = new Coral(
                    new CoralIOSim(), 
                    VirtualConstants.TROUGH_SWITCH_PORT
                );

                alga = new Alga(
                    new AlgaIOSim(), 
                    VirtualConstants.ALGA_UP_SWITCH_PORT,
                    VirtualConstants.ALGA_DOWN_SWITCH_PORT
                );
                break;
            default:
                drive = new Drive(
                    new ModuleIOReal(1),
                    new ModuleIOReal(2),
                    new ModuleIOReal(3),
                    new ModuleIOReal(4)
                );

                poseEstimator = new PoseEstimator(
                    new GyroIOReal(),
                    new CameraIOReal(),
                    drive
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
                    ), 
                    VirtualConstants.TROUGH_SWITCH_PORT
                );

                alga = new Alga(
                    new AlgaIOReal(
                        VirtualConstants.ALGA_ID_1,
                        VirtualConstants.ALGA_ID_2
                    ), 
                    VirtualConstants.ALGA_UP_SWITCH_PORT,
                    VirtualConstants.ALGA_DOWN_SWITCH_PORT
                );
                break;
        }

        drive.setPoseEstimator(poseEstimator);

        autoGenerator = new AutoRoutineGenerator(
            drive, 
            poseEstimator
        );
        autoChooser = new AutoChooser();
        
        configureButtonBindings();
        configureAutos();
    }

    private void configureButtonBindings() {

        // ————— drive ————— //

        drive.setDefaultCommand(
            new DriveWithVelocity(
                drive,
                poseEstimator,
                () -> -controller1.getLeftY(), // xboxcontroller is flipped
                () -> controller1.getLeftX(), 
                () -> controller1.getRightX()
            )
        );

        // manual position control
        // controller1.y().whileTrue(Commands.run(() -> drive.runPosition(new Pose2d(3, 2.5, new Rotation2d(-Math.PI/2))), drive));
        // controller1.x().whileTrue(Commands.run(() -> drive.runPosition(new Pose2d(5, 0, new Rotation2d(Math.PI))), drive));
        // controller1.a().whileTrue(Commands.run(() -> drive.runPosition(new Pose2d(1, 1, new Rotation2d())), drive));
        // controller1.b().onTrue(new InstantCommand(() -> drive.resetPoseEstimator(new Pose2d(1, 1, new Rotation2d()))));

        // ————— coral ————— //

        // // elevator
        // controller2.y().onTrue(coral.getSetElevatorCommand(Rotations.of(2.1875)));
        // controller2.a().onTrue(coral.getSetElevatorCommand(Rotations.of(0.5)));

        // arm
        controller2.x().onTrue(coral.getSetArmCommand(Rotations.of(0.2)));
        controller2.b().onTrue(coral.getSetArmCommand(Rotations.of(0)));

        // wrist
        // controller2.leftBumper().onTrue(new InstantCommand(() -> coral.setWristPosition(Rotations.of(1))));
        // controller2.rightBumper().onTrue(new InstantCommand(() -> coral.setWristPosition(Rotations.of(0))));
        
        // claw
        // controller2.x().onTrue(new InstantCommand(() -> coral.setClawPosition(false)));
        // controller2.b().onTrue(new InstantCommand(() -> coral.setClawPosition(true))); // open
        // controller2.a().onTrue(new InstantCommand(() -> coral.setClawVoltage(Volts.of(0))));

        // controller2.x().whileTrue(coral.elevatorSysIdFull());
        // controller2.y().whileTrue(coral.armSysIdFull());
        // controller2.b().whileTrue(coral.wristSysIdFull());
        // controller2.a().whileTrue(coral.clawSysIdFull());

        // ————— alga ————— // 

        // controller1.y().onTrue(alga.getUpCommand());
        // controller1.a().onTrue(alga.getDownCommand());
        // controller1.x().onTrue(alga.getIntakeCommand());
        // controller1.b().onTrue(alga.getOutputCommand());
        // controller1.rightBumper().onTrue(alga.getDownAtMatchStartCommand());
    }

    private void configureAutos(){
        autoChooser.addRoutine("2Meter", () -> autoGenerator.twoMeter());
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoralChoreo());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand(); // 2.5
    }
}