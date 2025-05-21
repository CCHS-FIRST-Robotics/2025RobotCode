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
        // if (coral.troughSensesCoral()) { // ! need to attach or find other solution
        //     return null;
        // }

        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_RUN)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.INTAKE_RUN))
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    }

    public Command prepL1() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L1);
    }

    public Command prepL1WithWait() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L1)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L1));
    }

    public Command runL1() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(0), Rotations.of(0));
    }

    public Command prepL2() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2);
    }

    public Command prepL2WithWait() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L2));
    }
    
    public Command runL2() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-0.3), Rotations.of(-0.08));
    }

    public Command runL2WithBackup() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-0.3), Rotations.of(-0.08))
        .alongWith(
            Commands.waitSeconds(1)
            .andThen(new InstantCommand(() -> drive.runVelocity(
                new ChassisSpeeds(
                    -0.1, //! tune 
                    0, 
                    0
                )   
            )).repeatedly().withTimeout(3))
        )
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    }

    public Command prepL3() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3);
    }

    public Command prepL3WithWait() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3)
        .andThen(coral.getWaitUntilCoralInPositionCommand(PhysicalConstants.CoralPositions.L3));
    }

    public Command runL3() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-0.3), Rotations.of(-0.1));
    }

    public Command runL3WithBackup() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-3), Rotations.of(-0.13)) 
        .alongWith(
            Commands.waitSeconds(1)
            .andThen(new InstantCommand(() -> drive.runVelocity(
                new ChassisSpeeds( 
                    -0.3, //! tune 
                    0, 
                    0
                )
            )).repeatedly().withTimeout(1))
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
        .andThen(coral.getSetArmVoltageCommand(Volts.of(0.25)));
    }

    public Command runL4WithBackup() {
        return coral.getLowerArmWithVoltageCommand(Volts.of(-1), Rotations.of(0.0815828125))
        .andThen(new InstantCommand(() -> drive.runVelocity(
            new ChassisSpeeds(
                -0.2, //! tune 
                0, 
                0
            )
        )).repeatedly().withTimeout(1.5))
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_PREP));
    }
}