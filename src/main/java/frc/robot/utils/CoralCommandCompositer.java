package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.coralIO.Coral;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.commands.DriveWithPosition;
import frc.robot.constants.*;

public class CoralCommandCompositer {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Coral coral;

    public CoralCommandCompositer(
        Drive drive,
        PoseEstimator poseEstimator,
        Coral coral
    ) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.coral = coral;
    }

    public Command prepIntake() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_1);
    }

    public Command runIntake() { 
        if(coral.troughSensesCoral()){
            return null;
        }

        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_2);
    }

    public Command prepL1() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L1);
    }

    public Command runL1() {
        return coral.getLowerArmWithVoltageCommand();
    }

    public Command prepL2() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2);
    }

    public Command runL2() {
        return coral.getLowerArmWithVoltageCommand();
    }

    public Command prepL3() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3);
    }

    public Command runL3() {
        return coral.getLowerArmWithVoltageCommand();
    }

    public Command prepL4() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4_1);
    }

    public Command runL4() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4_2)
        .andThen(new DriveWithPosition(drive, poseEstimator, new Transform2d(0, -1, new Rotation2d()))) // ! this is not backwards from the robot
        .andThen(coral.getLowerArmWithVoltageCommand());
    }
}