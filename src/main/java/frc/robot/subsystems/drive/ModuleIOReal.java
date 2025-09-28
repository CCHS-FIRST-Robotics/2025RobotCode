package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PhysicalConstants;

public class ModuleIOReal implements ModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    private final SparkMaxConfig turnConfig = new SparkMaxConfig();
    private final SimpleMotorFeedforward driveFeedforward;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandMag

    // for drive, pid units are in V/rpm, ff units are the normal V/(rad per second)
    private double driveKp = 0.000062333;     
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0.03422;
    private double driveKv = 0.13259;
    private double driveKa = 0.025003;

    private double turnKp = 8 / (2 * Math.PI);
    private double turnKi = 0;
    private double turnKd = 1.5 / (2 * Math.PI);

    private AngularVelocity prevDriveVelocity = RadiansPerSecond.of(0.0);

    public ModuleIOReal(int index) {
        // our drive and turn motor CAN IDs are set as 11, 12 for module 1; 21, 22 for module 2; and so on
        driveMotor = new SparkMax(10 * index + 1, MotorType.kBrushless);
        turnMotor = new SparkMax(10 * index + 2, MotorType.kBrushless);

        // start config
        driveMotor.setCANTimeout(500);
        turnMotor.setCANTimeout(500);
        
        // encoders
        driveEncoder = driveMotor.getEncoder();
        turnRelativeEncoder = turnMotor.getEncoder();
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder.setPosition(0.0);
        turnRelativeEncoder.setPosition(0.0);
        turnConfig.absoluteEncoder.inverted(true);
        turnConfig.absoluteEncoder.averageDepth(2);
        
        // pid 
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(driveKp, driveKi, driveKd, ClosedLoopSlot.kSlot0);
        driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(turnKp, turnKi, turnKd, ClosedLoopSlot.kSlot0);
        turnConfig.closedLoop.positionWrappingEnabled(true);
        turnConfig.closedLoop.positionWrappingMinInput(0);
        turnConfig.closedLoop.positionWrappingMaxInput(1);
        
        // miscellaneous settings
        driveConfig.signals.primaryEncoderVelocityPeriodMs(10);
        turnConfig.signals.absoluteEncoderPositionPeriodMs(20);
        
        driveConfig.encoder.quadratureMeasurementPeriod(10);
        driveConfig.encoder.quadratureAverageDepth(2);
        turnConfig.encoder.quadratureMeasurementPeriod(10);
        turnConfig.encoder.quadratureAverageDepth(2);
        
        driveConfig.smartCurrentLimit(30);
        driveConfig.voltageCompensation(12);
        turnConfig.smartCurrentLimit(20);
        turnConfig.voltageCompensation(12);

        turnConfig.inverted(true);

        driveConfig.idleMode(IdleMode.kBrake);
        turnConfig.idleMode(IdleMode.kBrake);

        // stop config
        driveMotor.setCANTimeout(0);
        turnMotor.setCANTimeout(0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void setDriveVoltage(Voltage volts) {
        driveMotor.setVoltage(volts.in(Volts));
    }
    
    @Override
    public void setTurnVoltage(Voltage volts) {
        turnMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveMotor.getClosedLoopController().setReference(
            velocity.in(Rotations.per(Minute)) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION,
            SparkMax.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            driveFeedforward.calculateWithVelocities(
                prevDriveVelocity.in(RadiansPerSecond),
                velocity.in(RadiansPerSecond)
            )
        );

        prevDriveVelocity = velocity;
    }

    @Override
    public void setTurnPosition(Angle position) {
        turnMotor.getClosedLoopController().setReference(
            position.in(Rotations),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.drivePosition = Rotations.of(driveEncoder.getPosition() / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION).in(Radians);
        inputs.driveVelocity = Rotations.per(Minute).of(driveEncoder.getVelocity() / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION).in(RadiansPerSecond);
        inputs.driveTemperature = driveMotor.getMotorTemperature();
        
        inputs.turnVoltage = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrent = turnMotor.getOutputCurrent();
        inputs.turnPosition = turnAbsoluteEncoder.getPosition();
        inputs.turnVelocity = turnAbsoluteEncoder.getVelocity();
        inputs.turnTemperature = turnMotor.getMotorTemperature();
    }
}