package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.VirtualConstants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1)
    );

    private double driveKp = 0.00015;
    private double driveKi = 0;
    private double driveKd = 0.0;
    private double driveKs = 0;
    private double driveKv = 0.1362; // From NEO datasheet (473kV): 0.136194 V/(rad/s) - https://www.wolframalpha.com/input?i=1%2F%28473+*+2pi%2F60%29+*+%2850.0+%2F+14.0%29+*+%2817.0+%2F+27.0%29+*+%2845.0+%2F+15.0%29
    private double driveKa = 0.0148;

    private double turnKp = 8;
    private double turnKi = 0; 
    private double turnKd = 1.5;

    private final PIDController drivePID = new PIDController(driveKp, driveKi, driveKd);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
    private final PIDController turnPID = new PIDController(turnKp, turnKi, turnKd);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    
    AngularVelocity prevVelocity = RadiansPerSecond.of(0.0);

    public ModuleIOSim() {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveKv, driveKa), 
            DCMotor.getNEO(1), 
            1, 1
        );
        
        turnPID.enableContinuousInput(0, 1);
    }

    @Override
    public void setDriveVoltage(Voltage volts) {
        driveAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0); // ! why is clamp there
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(Voltage volts) {
        turnAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0); // ! why is clamp there
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        double volts = drivePID.calculate(
            driveSim.getAngularVelocityRPM() / 60,
            velocity.in(RotationsPerSecond))
            + driveFF.calculateWithVelocities(prevVelocity.in(RotationsPerSecond), velocity.in(RotationsPerSecond)
        );

        setDriveVoltage(Volts.of(volts));
        prevVelocity = velocity;
    }

    @Override
    public void setTurnPosition(Angle position) {
        position = Rotations.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));
        Angle currentPosition = Rotations.of(MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0, 2 * Math.PI));

        double volts = turnPID.calculate(
            currentPosition.in(Rotations),
            position.in(Rotations)
        );

        setTurnVoltage(Volts.of(volts));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(VirtualConstants.PERIOD);
        turnSim.update(VirtualConstants.PERIOD);

        inputs.driveVoltage = driveAppliedVolts;
        inputs.driveCurrent = driveSim.getCurrentDrawAmps();
        inputs.drivePosition = driveSim.getAngularPositionRotations();
        inputs.driveVelocity = driveSim.getAngularVelocityRPM() / 60;

        inputs.turnVoltage = turnAppliedVolts;
        inputs.turnCurrent = turnSim.getCurrentDrawAmps();
        inputs.turnPosition = turnSim.getAngularPositionRotations();
    }
}