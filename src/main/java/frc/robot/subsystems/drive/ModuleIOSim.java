package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;

import org.littletonrobotics.junction.Logger;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim = new DCMotorSim( // 
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), // ! ask colin where he got 0.004 from
        DCMotor.getNEO(1)
    );

    private int index;

    private double driveKp = 0.5;
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = (1 / (473d / 60d)) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION; // neo kV = 473 rpm/V (datasheet)
    private double driveKa = 0.0929; // ! change this later when sysID exists

    private double turnKp = 2;
    private double turnKi = 0;
    private double turnKd = 0;

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
        Logger.recordOutput("Thing1", driveSim.getAngularVelocityRPM() / 60);
        Logger.recordOutput("Thing2", velocity.in(RotationsPerSecond));
        Logger.recordOutput("Thing3", driveFF.calculateWithVelocities(prevVelocity.in(RotationsPerSecond), velocity.in(RotationsPerSecond)));

        setDriveVoltage(Volts.of(volts));
        prevVelocity = velocity;

        Logger.recordOutput("modules/" + Integer.toString(index) + "DriveVelocitySetpoint", velocity.in(RotationsPerSecond));
        Logger.recordOutput("modules/" + Integer.toString(index) + "DriveVoltageSetpoint", volts);
    }

    @Override
    public void setTurnPosition(Angle position) {
        position = Radians.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));
        Angle currentPosition = Radians.of(MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0, 2 * Math.PI));

        double volts = turnPID.calculate(
            currentPosition.in(Rotations),
            position.in(Rotations)
        );

        setTurnVoltage(Volts.of(volts));

        Logger.recordOutput("modules/" + Integer.toString(index) + "TurnPositionSetpoint", position.in(Rotations));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(VirtualConstants.PERIOD);
        turnSim.update(VirtualConstants.PERIOD);

        inputs.driveVoltage = driveAppliedVolts;
        inputs.driveCurrent = driveSim.getCurrentDrawAmps();
        inputs.drivePosition = driveSim.getAngularPositionRad();
        inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec();

        inputs.turnVoltage = turnAppliedVolts;
        inputs.turnCurrent = turnSim.getCurrentDrawAmps();
        inputs.turnPosition = turnSim.getAngularPositionRotations();
    }
}