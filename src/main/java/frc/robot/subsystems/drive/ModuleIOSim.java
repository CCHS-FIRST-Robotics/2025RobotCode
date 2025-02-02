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

    // private double driveKp = 0.00015;
    // private double driveKi = 0;
    // private double driveKd = 0;
    // private double driveKs = 0;
    // private double driveKv = (1 / (473d / 60d)) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION; // neo kV = 473 rpm/V (datasheet)
    // private double driveKa = 0.0929; // ! change this later when sysID exists

    public double driveKp = 0.00015; // 0.00015
    public double driveKd = 0.0;
    public double driveKi = 0.000000; // 0.000008
    public double driveKs = 0.0; // .19
    public double driveKv = 0.1362; // From NEO datasheet (473kV): 0.136194 V/(rad/s) -
                                    // https://www.wolframalpha.com/input?i=1%2F%28473+*+2pi%2F60%29+*+%2850.0+%2F+14.0%29+*+%2817.0+%2F+27.0%29+*+%2845.0+%2F+15.0%29
    public double driveKa = 0.0148;

    private double turnKp = 8 / (2 * Math.PI);
    private double turnKi = 0;
    private double turnKd = 1.5 / (2 * Math.PI);

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
            driveSim.getAngularVelocityRadPerSec(),
            velocity.in(RadiansPerSecond))
            + driveFF.calculateWithVelocities(prevVelocity.in(RadiansPerSecond), velocity.in(RadiansPerSecond)
        );

        setDriveVoltage(Volts.of(volts));
        driveAppliedVolts = volts;
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
        turnAppliedVolts = volts;

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