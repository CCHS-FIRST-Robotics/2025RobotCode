package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
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

    public void runState(SwerveModuleState state) {
        // state.optimize(getWrappedAngle());
        Logger.recordOutput("OptimizedStates/" + Integer.toString(index), state);
        io.setTurnPosition(Rotations.of(state.angle.getRotations()));
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / PhysicalConstants.WHEEL_RADIUS.in(Meters)));
    }

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    // ————— functions for odometry ————— //

    public double getDistanceTraveled() {
        return inputs.drivePosition * PhysicalConstants.WHEEL_RADIUS.in(Meters);
    }

    public Rotation2d getWrappedAngle() {
        return new Rotation2d(Rotations.of(inputs.turnPosition).in(Radians));
        // return new Rotation2d(MathUtil.angleModulus(Rotations.of(inputs.turnPosition).in(Radians)));
    }
}