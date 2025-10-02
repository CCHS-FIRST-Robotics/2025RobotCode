package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.coralIO.Coral;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.constants.*;

public class CoralCommandCompositer {
    private final Drive drive;
    private final Coral coral;

    public CoralCommandCompositer(
        Drive drive,
        PoseEstimator poseEstimator,
        Coral coral
    ) {
        this.drive = drive;
        this.coral = coral;
    }

    public Command prepIntake() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP);
    }

    public Command runIntake() { 
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_RUN)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.INTAKE_RUN))
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    }

    // public Command prepL2() {
    //     return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2);
    // }

    // public Command prepL2WithWait() {
    //     return prepL2()
    //     .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L2));
    // }
    
    // public Command runL2() {
    //     return coral.getLowerArmWithVoltageCommand(Volts.of(-0.3), Rotations.of(-0.08));
    // }

    // public Command runL2WithBackup() {
    //     return runL2()
    //     .alongWith(
    //         Commands.waitSeconds(1)
    //         .andThen(new InstantCommand(() -> drive.runVelocity(
    //             new ChassisSpeeds(
    //                 -0.1, // ! tune 
    //                 0, 
    //                 0
    //             )   
    //         )).repeatedly().withTimeout(3))
    //     )
    //     .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    // }

    public Command prepL3() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3);
    }

    public Command prepL3WithWait() {
        return prepL3()
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L3));
    }

    public Command runL3() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-0.3), Rotations.of(-0.1));
    }

    public Command runL3WithBackup() { // ! 
        return runL3()
        .alongWith(
            Commands.waitSeconds(1)
            .andThen(new InstantCommand(() -> drive.runVelocity(
                new ChassisSpeeds( 
                    -0.1, // ! tune 
                    0, 
                    0
                )
            )).repeatedly().withTimeout(2))
        )
        .andThen(coral.getSetElevatorCommand(PhysicalConstants.CoralPositions.INTAKE_PREP[0]))
        .andThen(Commands.waitSeconds(0.8))
        .andThen(coral.getSetArmCommand(PhysicalConstants.CoralPositions.INTAKE_PREP[1]));
    }

    public Command prepL4() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4);
    }

    public Command prepL4WithWait() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L4));
    }

    public Command runL4() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-0.25), Rotations.of(0.0673828125))
        .andThen(coral.getSetArmVoltageCommand(Volts.of(0.25))); // holding voltage
    }

    public Command runL4WithBackup() {
        return runL4()
        .andThen(new InstantCommand(() -> drive.runVelocity(
            new ChassisSpeeds(
                -0.2, // ! tune 
                0, 
                0
            )
        )).repeatedly().withTimeout(1.5))
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    }
}