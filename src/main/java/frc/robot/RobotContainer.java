// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.math.geometry.*;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;
import frc.robot.subsystems.coralIO.*;
import frc.robot.utils.*;

public class RobotContainer {
    private final CommandXboxController xboxController = new CommandXboxController(VirtualConstants.XBOX_CONTROLLER_PORT_1);
    // private final CommandGenericHID coralController = new CommandGenericHID(VirtualConstants.CORAL_CONTROLLER_PORT_3);

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Coral coral;

    private final CoralCommandCompositer coralCommandCompositer;
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
                    new CameraIOPhotonVision[] {
                        new CameraIOPhotonVision(0),
                        new CameraIOPhotonVision(1)
                    },
                    drive
                );

                coral = new Coral(
                    new CoralIOReal(
                        VirtualConstants.ELEVATOR_ID, 
                        VirtualConstants.ELEVATOR_CANCODER_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.ARM_CANCODER_ID
                    )
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
                    new CameraIOPhotonVision[] {
                        new CameraIOPhotonVision(0),
                        new CameraIOPhotonVision(1)
                    },
                    drive
                );

                coral = new Coral(
                    new CoralIOSim()
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
                    new CameraIOPhotonVision[] {
                        new CameraIOPhotonVision(0),
                        new CameraIOPhotonVision(1)
                    },
                    drive
                );

                coral = new Coral(
                    new CoralIOReal(
                        VirtualConstants.ELEVATOR_ID, 
                        VirtualConstants.ELEVATOR_CANCODER_ID, 
                        VirtualConstants.ARM_ID, 
                        VirtualConstants.ARM_CANCODER_ID
                    )
                );
                break;
        }

        drive.setPoseEstimator(poseEstimator);
        poseEstimator.resetPosition(new Pose2d());

        coralCommandCompositer = new CoralCommandCompositer(drive, poseEstimator, coral);
        autoGenerator = new AutoRoutineGenerator(drive, poseEstimator, coralCommandCompositer);
        autoChooser = new AutoChooser();
        
        configureButtonBindings();
        configureAutos();
    }

    private void configureButtonBindings() {

        // ————— drive ————— //

        // drive
        drive.setDefaultCommand(
            new DriveWithVelocity(
                drive,
                poseEstimator,
                () -> -xboxController.getLeftY(), // xboxcontroller is flipped
                () -> xboxController.getLeftX(), 
                () -> xboxController.getRightX()
            )
        );

        // x-lock
        // xboxController.x().whileTrue(
        //     Commands.run(() -> drive.runCharacterization(
        //         new Voltage[] {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)}, 
        //         new Angle[] {Rotations.of(0.125), Rotations.of(0.325), Rotations.of(0.325), Rotations.of(0.125)})
        //     )
        // );

        // ! also testing with multiple apriltags to see if multitagpnp is working

        // ! try resetting pose to 0 and 180 degrees, see if the gyro rotation is really taken care of without even rezeroing the gyro. if so, I don't need to worry about relative gyro whatever
        xboxController.leftTrigger().onTrue(new InstantCommand(() -> poseEstimator.resetPosition(new Pose2d(0, 0, new Rotation2d()))));
        xboxController.rightTrigger().onTrue(new InstantCommand(() -> poseEstimator.resetPosition(new Pose2d(0, 0, new Rotation2d(Degrees.of(180))))));

        // xboxController.y().onTrue(coralCommandCompositer.prepL4());
        // xboxController.b().onTrue(coralCommandCompositer.prepIntake());
        // xboxController.a().onTrue(coralCommandCompositer.runIntake());

        // // reset pose based on vision
        // xboxController.rightBumper().onTrue(new InstantCommand(() -> poseEstimator.resetPosition(poseEstimator.getPose())));
        
        // // drive to L4 // ! current position is not theoretically correct
        // xboxController.rightTrigger().whileTrue(new DriveWithPosition(drive, poseEstimator, new Pose2d(3, 4.0259, new Rotation2d())));

        // ————— coral ————— //

        // branch positions
        // coralController.button(4).onTrue(coralCommandCompositer.prepIntake());
        // coralController.button(5).onTrue(coralCommandCompositer.runIntake());
        // // coralController.button(9).onTrue(coralCommandCompositer.prepL2());
        // // coralController.button(10).onTrue(coralCommandCompositer.runL2());
        // coralController.button(17).onTrue(coralCommandCompositer.prepL3());
        // coralController.button(18).onTrue(coralCommandCompositer.runL3WithBackup());
        // coralController.button(19).onTrue(coralCommandCompositer.prepL4());
        // coralController.button(20).onTrue(coralCommandCompositer.runL4());

        // emergency stop
        // coralController.button(21).onTrue(
        //     coral.getSetElevatorVoltageCommand(Volts.of(0))
        //     .andThen(coral.getSetArmVoltageCommand(Volts.of(0)))
        // );
    }

    private void configureAutos() {
        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());
        autoChooser.addRoutine("1CoralL4", () -> autoGenerator.oneCoralL4());

        autoChooser.select("Back Up");

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}