// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.units.measure.*;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.utils.*;

public class RobotContainer {
    private final CommandXboxController xboxController1 = new CommandXboxController(VirtualConstants.CONTROLLER_PORT_1);
    private final CommandXboxController xboxController2 = new CommandXboxController(VirtualConstants.CONTROLLER_PORT_2);
    private final CommandGenericHID coralController = new CommandGenericHID(VirtualConstants.CONTROLLER_PORT_3);

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Coral coral;

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
                        VirtualConstants.ARM_CANCODER_ID
                    ), 
                    VirtualConstants.TROUGH_SWITCH_PORT
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
                        VirtualConstants.ARM_CANCODER_ID
                    ), 
                    VirtualConstants.TROUGH_SWITCH_PORT
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
                () -> -xboxController1.getLeftY(), // xboxcontroller is flipped
                () -> xboxController1.getLeftX(), 
                () -> xboxController1.getRightX()
            )
        );

        // x-lock
        xboxController1.rightTrigger().whileTrue(
            Commands.run(() -> drive.runCharacterization(
                new Voltage[] {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)}, 
                new Angle[] {Rotations.of(0.125), Rotations.of(0.325), Rotations.of(0.325), Rotations.of(0.125)})
            )
        );

        // ————— coral ————— //

        // elevator
        xboxController2.y().onTrue(coral.getSetElevatorCommand(PhysicalConstants.ELEVATOR_MAX_ROTATIONS));
        xboxController2.a().onTrue(coral.getSetElevatorCommand(PhysicalConstants.ELEVATOR_MIN_ROTATIONS));

        // xboxController2.y().onTrue(coral.getSetElevatorVoltageCommand(Volts.of(1)));
        // xboxController2.a().onTrue(coral.getSetElevatorVoltageCommand(Volts.of(-1)));

        // arm
        xboxController2.x().onTrue(coral.getSetArmCommand(PhysicalConstants.ARM_MAX_ROTATIONS));
        xboxController2.b().onTrue(coral.getSetArmCommand(PhysicalConstants.ARM_MIN_ROTATIONS));

        // xboxController2.x().onTrue(coral.getSetArmVoltageCommand(Volts.of(1)));
        // xboxController2.b().onTrue(coral.getSetArmVoltageCommand(Volts.of(-1)));

        // // reef positions
        // if ((DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red).equals(Alliance.Red)) {
        //     coralController.button(1).onTrue(new DriveWithApriltag(drive, poseEstimator, 6, true));
        //     coralController.button(2).onTrue(new DriveWithApriltag(drive, poseEstimator, 6, false));
        //     coralController.button(3).onTrue(new DriveWithApriltag(drive, poseEstimator, 7, true));
        //     coralController.button(6).onTrue(new DriveWithApriltag(drive, poseEstimator, 7, false));
        //     coralController.button(7).onTrue(new DriveWithApriltag(drive, poseEstimator, 8, true));
        //     coralController.button(8).onTrue(new DriveWithApriltag(drive, poseEstimator, 8, false));
        //     coralController.button(11).onTrue(new DriveWithApriltag(drive, poseEstimator, 9, true));
        //     coralController.button(13).onTrue(new DriveWithApriltag(drive, poseEstimator, 9, false));
        //     coralController.button(15).onTrue(new DriveWithApriltag(drive, poseEstimator, 10, true));
        //     coralController.button(12).onTrue(new DriveWithApriltag(drive, poseEstimator, 10, false));
        //     coralController.button(14).onTrue(new DriveWithApriltag(drive, poseEstimator, 11, true));
        //     coralController.button(16).onTrue(new DriveWithApriltag(drive, poseEstimator, 11, false));
        // } else {
        //     coralController.button(1).onTrue(new DriveWithApriltag(drive, poseEstimator, 17, true));
        //     coralController.button(2).onTrue(new DriveWithApriltag(drive, poseEstimator, 17, false));
        //     coralController.button(3).onTrue(new DriveWithApriltag(drive, poseEstimator, 18, true));
        //     coralController.button(6).onTrue(new DriveWithApriltag(drive, poseEstimator, 18, false));
        //     coralController.button(7).onTrue(new DriveWithApriltag(drive, poseEstimator, 19, true));
        //     coralController.button(8).onTrue(new DriveWithApriltag(drive, poseEstimator, 19, false));
        //     coralController.button(11).onTrue(new DriveWithApriltag(drive, poseEstimator, 20, true));
        //     coralController.button(13).onTrue(new DriveWithApriltag(drive, poseEstimator, 20, false));
        //     coralController.button(15).onTrue(new DriveWithApriltag(drive, poseEstimator, 21, true));
        //     coralController.button(12).onTrue(new DriveWithApriltag(drive, poseEstimator, 21, false));
        //     coralController.button(14).onTrue(new DriveWithApriltag(drive, poseEstimator, 22, true));
        //     coralController.button(16).onTrue(new DriveWithApriltag(drive, poseEstimator, 22, false));
        // }

        // // stalk positions
        // coralController.button(4 ).onTrue(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE));
        // coralController.button(5 ).onTrue(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2));
        // coralController.button(9 ).onTrue(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3));
        // coralController.button(10).onTrue(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4));
    }

    private void configureAutos(){
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());
        autoChooser.addRoutine("2Meter", () -> autoGenerator.twoMeter());
        autoChooser.addRoutine("2Coral", () -> autoGenerator.twoCoralChoreo());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand(); // 2.5
    }
}