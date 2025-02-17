package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.PhysicalConstants;

public class Module {
    private final ModuleIO io;
    private final int index;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("drive/module" + Integer.toString(index), inputs);
    }

    // ————— functions for running modules ————— //

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    public void runCharacterization(Voltage volts){
        io.setTurnPosition(Rotations.of(0));
        io.setDriveVoltage(volts);
    }

    public void runState(SwerveModuleState state) {
        state.optimize(getAngle());
        io.setTurnPosition(Rotations.of(state.angle.getRotations()));
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / PhysicalConstants.WHEEL_RADIUS.in(Meters)));
    }

    // ————— functions for odometry ————— //

    public double getDistance() {
        return inputs.drivePosition * PhysicalConstants.WHEEL_RADIUS.in(Meters);
    }

    public double getVelocity() {
        return inputs.driveVelocity * PhysicalConstants.WHEEL_RADIUS.in(Meters);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(Rotations.of(inputs.turnPosition).in(Radians));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }
}