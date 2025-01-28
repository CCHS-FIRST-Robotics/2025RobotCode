package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import frc.robot.constants.HardwareConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;

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
        state.optimize(getWrappedAngle()); // ! what angle does it actually want
        io.setTurnPosition(Rotations.of(state.angle.getRotations()));
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / HardwareConstants.WHEEL_RADIUS.in(Meters)));
    }

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    // ————— functions for odometry ————— //

    public double getDistanceTraveled() {
        return inputs.drivePosition * HardwareConstants.WHEEL_RADIUS.in(Meters);
    }

    public Rotation2d getWrappedAngle() {
        return new Rotation2d(MathUtil.angleModulus(Rotations.of(inputs.turnPosition).in(Radians)));
    }
}