package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.VirtualConstants;

import org.littletonrobotics.junction.Logger;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1)
    );

    private int index;

    private double driveKp = 0;
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    // 0.8557



    private double driveKv = 0.8557; // neo kV = 473 rpm/V (from datasheet), then adjust for gearing
    private double driveKa = 0.0929911425; // 0.0148; V/(rad/s^2), Vs^2/rad

    private double turnKp = 2 * 2 * Math.PI; // !
    private double turnKi = 0 * 2 * Math.PI; 
    private double turnKd = 0 * 2 * Math.PI; // !

    private final PIDController drivePID = new PIDController(driveKp, driveKi, driveKd);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
    private final PIDController turnPID = new PIDController(turnKp, turnKi, turnKd);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    
    AngularVelocity prevVelocity = RotationsPerSecond.of(0.0);

    public ModuleIOSim(int index) {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveKv, driveKa), 
            DCMotor.getNEO(1), 
            1, 1
        );
        this.index = index;
        
        turnPID.enableContinuousInput(0, 1);
    }

    @Override
    public void setDriveVoltage(Voltage volts) {
        driveSim.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void setTurnVoltage(Voltage volts) {
        turnSim.setInputVoltage(volts.in(Volts));
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

        Logger.recordOutput("modules/ModuleSimVelocitySetpoint" + Integer.toString(index), velocity.in(RotationsPerSecond));
        Logger.recordOutput("modules/ModuleSimVoltageSetpoint" + Integer.toString(index), volts);
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