package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim = new DCMotorSim( // 
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.004,  // ! ask colin where he got this from
            1
        ),
        DCMotor.getNEO(1)
    );

    // for drive, pid units are in V/rpm, ff units are the normal V/(rad per second)
    private double driveKp = 0.00015 * 2d * Math.PI / 60d ; 
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = 1/(473d * 2d * Math.PI / 60d) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION; // neo kV = 473 rpm/V (from datasheet)    
    private double driveKa = 0.020864; // ! replace after sysid

    private double turnKp = 8 / (2 * Math.PI);
    private double turnKi = 0;
    private double turnKd = 1.5 / (2 * Math.PI);

    private final PIDController drivePID = new PIDController(driveKp, driveKi, driveKd);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
    private final PIDController turnPID = new PIDController(turnKp, turnKi, turnKd);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private AngularVelocity prevVelocity = RotationsPerSecond.of(0.0);

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
    }

    @Override
    public void setTurnPosition(Angle position) {
        double volts = turnPID.calculate(
            turnSim.getAngularPositionRotations(),
            position.in(Rotations)
        );

        setTurnVoltage(Volts.of(volts));
        turnAppliedVolts = volts;
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