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

    public Command intake() {
        if(coral.troughSensesCoral()){
            return null;
        }

        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_1)
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.INTAKE_2));
    }

    public Command L1() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L1)
        .andThen(coral.getLowerArmWithVoltageCommand());
    }

    public Command L2() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L2)
        .andThen(coral.getLowerArmWithVoltageCommand());
    }

    public Command L3() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L3)
        .andThen(coral.getLowerArmWithVoltageCommand());
    }

    public Command L4() {
        return coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4_1)
        .andThen(coral.getSetCoralPositionCommand(PhysicalConstants.CoralPositions.L4_2))
        .andThen(new DriveWithPosition(drive, poseEstimator, new Transform2d(0, -1, new Rotation2d()))); // ! this is not backwards from the robot
    }
}