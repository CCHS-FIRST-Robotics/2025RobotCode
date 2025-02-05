package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
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

    public void runState(SwerveModuleState state) {
        state.optimize(getAngle()); // ! bugged
        io.setTurnPosition(Rotations.of(state.angle.getRotations()));
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / PhysicalConstants.WHEEL_RADIUS.in(Meters)));
    }

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    public void characterize(Voltage volts){
        io.setTurnPosition(Rotations.of(0));
        io.setDriveVoltage(volts);
    }

    // ————— functions for odometry ————— //

    public double getDistance() {
        return inputs.drivePosition * PhysicalConstants.WHEEL_RADIUS.in(Meters);
    }

    public Rotation2d getAngle() {
        // ! bugged af
        return new Rotation2d(MathUtil.angleModulus(Rotations.of(inputs.turnPosition).in(Radians)));
    }
}