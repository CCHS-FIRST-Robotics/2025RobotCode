package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.TunableNumber;

public class ModuleIOSim implements ModuleIO {

    public final TunableNumber driveKp = new TunableNumber("driveKp", 0.00015);
    public final TunableNumber driveKi = new TunableNumber("driveKi", 0.0);
    public final TunableNumber driveKd = new TunableNumber("driveKd", 0.0);
    public final TunableNumber driveKs = new TunableNumber("driveKs", 0.0);
    public final TunableNumber driveKv = new TunableNumber("driveKv", 0.1362);
    public final TunableNumber driveKa = new TunableNumber("driveKa", 0.0148);

    public final TunableNumber turnKp = new TunableNumber("turnKp", 8);
    public final TunableNumber turnKi = new TunableNumber("turnKi", 0.0);
    public final TunableNumber turnKd = new TunableNumber("turnKd", 1.5);

    PIDController drivePID = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
    PIDController turnPID = new PIDController(turnKp.get(), turnKi.get(), turnKd.get());
    SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs.get(), driveKv.get(), driveKa.get());

    AngularVelocity prevVelocity = RadiansPerSecond.of(0.0);

    // private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 /
    // 27.0) * (45.0 / 15.0);
    // private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private DCMotorSim driveSim;

    private DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1)
    );

    private Rotation2d turnAbsoluteInitPosition = new Rotation2d(0);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        System.out.println("[Init] Creating ModuleIOSim");

        turnPID.enableContinuousInput(0, 1);
        driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveKv.get(), driveKa.get()), DCMotor.getNEO(1), 1);
        
    }

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.PERIOD);
        turnSim.update(Constants.PERIOD);

        inputs.drivePositionRad = Radians.of(driveSim.getAngularPositionRad());
        inputs.driveVelocityRadPerSec = RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec());
        inputs.driveAppliedVolts = Volts.of(driveAppliedVolts);
        inputs.driveAverageBusVoltage = Volts.of(12);
        inputs.driveCurrentAmps = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

        inputs.turnAbsolutePositionRad = Radians.of(
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition).getRadians());
        inputs.turnPositionRad = Radians.of(new Rotation2d(turnSim.getAngularPositionRad()).getRadians());
        inputs.turnVelocityRadPerSec = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVolts = Volts.of(turnAppliedVolts);
        inputs.turnAverageBusVoltage = Volts.of(12);
        inputs.turnCurrentAmps = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));

        if (driveKp.hasChanged() || driveKi.hasChanged() || driveKd.hasChanged()
            || turnKp.hasChanged() || turnKi.hasChanged() || turnKd.hasChanged()) {
                drivePID.setPID(driveKp.get(), driveKi.get(), driveKd.get());
                turnPID.setPID(turnKp.get(), turnKi.get(), turnKd.get());
            }
            
    }

    public void setDriveVoltage(Voltage volts) {
        driveAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(Voltage volts) {
        turnAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    public void setDriveVelocity(AngularVelocity velocity) {
        // velocity = velocity.times(driveAfterEncoderReduction);

        double volts = drivePID.calculate(
                RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec()).in(Rotations.per(Minute)),
                velocity.in(Rotations.per(Minute)))
                + driveFF.calculate(prevVelocity.in(RadiansPerSecond), velocity.in(RadiansPerSecond));

        prevVelocity = velocity;
        setDriveVoltage(Volts.of(volts));
        // driveSim.setState(driveSim.getAngularPositionRad(),
        // velocityRadPerSec.in(RadiansPerSecond));
    }

    public void setTurnPosition(Angle position) {
        position = Radians.of(
                MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));

        Angle currentPosition = Radians.of(
                MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0, 2 * Math.PI));

        double volts = turnPID.calculate(
                currentPosition.in(Rotations),
                position.in(Rotations));

        setTurnVoltage(Volts.of(volts));
        // turnSim.setState(turnRelativePositionRad,
        // turnSim.getAngularVelocityRadPerSec());
    }
}
