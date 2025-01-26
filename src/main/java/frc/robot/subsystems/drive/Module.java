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

        io.setDriveBrakeMode(true);
        io.setTurnBrakeMode(false);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("drive/module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState runState(SwerveModuleState state) {
        state.optimize(getAngle()); // optimize which way the wheel turns
        io.setTurnPosition(Rotations.of(state.angle.getRotations()));

        // state.speedMetersPerSecond *= Math.cos( // dot product of the state speed and the turn error
        //     inputs.turnAbsolutePositionRad - state.angle.getRadians() // ! idrk how necessary this is
        // );
        
        // ! uhhh see if this is important
        // // constrain velocity based on voltage and previous velocity using motor dynamics
        // state.speedMetersPerSecond = MathUtil.clamp(
        //     state.speedMetersPerSecond,
        //     getMaxVelocity(-inputs.driveAverageBusVoltage,
        //         prevState.speedMetersPerSecond / HardwareConstants.WHEEL_RADIUS.in(Meters), Constants.PERIOD,
        //         ModuleIO.driveKv, 
        //         ModuleIO.driveKa
        //     ) * HardwareConstants.WHEEL_RADIUS.in(Meters),
        //     getMaxVelocity(inputs.driveAverageBusVoltage,
        //         prevState.speedMetersPerSecond / HardwareConstants.WHEEL_RADIUS.in(Meters), Constants.PERIOD,
        //         ModuleIO.driveKv, 
        //         ModuleIO.driveKa
        //     ) * HardwareConstants.WHEEL_RADIUS.in(Meters)
        // );

        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / HardwareConstants.WHEEL_RADIUS.in(Meters)));
        return state;
    }

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    // ! make this its own separate input
    public double getDistanceTraveled() {
        return inputs.drivePositionRad * HardwareConstants.WHEEL_RADIUS.in(Meters);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }
}