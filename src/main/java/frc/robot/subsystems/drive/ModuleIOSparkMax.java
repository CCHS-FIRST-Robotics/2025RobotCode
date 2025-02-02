package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PhysicalConstants;

public class ModuleIOSparkMax implements ModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    private final SparkMaxConfig turnConfig = new SparkMaxConfig();
    private final SimpleMotorFeedforward driveFeedforward;

    private final RelativeEncoder driveEncoder; // NEO
    private final RelativeEncoder turnRelativeEncoder; // NEO
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder
    
    // for drive, pid units are in V/rpm, ff units are the normal V/(rad per second)
    private double driveKp = 0.00015 * 2d * Math.PI / 60d ; 
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = 1/(473d * 2d * Math.PI / 60d) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION; // neo kV = 473 rpm/V (from datasheet)    
    private double driveKa = 0.020864; // ! colin came up with this; replace it later

    private double turnKp = 8 / (2 * Math.PI);
    private double turnKi = 0;
    private double turnKd = 1.5 / (2 * Math.PI);

    private AngularVelocity prevDriveVelocity = RadiansPerSecond.of(0.0);

    public ModuleIOSparkMax(int index) {
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
        
        // miscellaneous settings // ! go through these at some point
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

        // ! see about using this
        // driveConfig.encoder.positionConversionFactor(1 / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION);
        // driveConfig.encoder.velocityConversionFactor(1 / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION);
        // turnConfig.encoder.positionConversionFactor(1 / PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION);
        // turnConfig.encoder.velocityConversionFactor(1 / PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION);
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
        // adjust from [-PI, PI] to [0, 2PI]  // ! out of curiosity, why
        position = Radians.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI)); 
        
        turnMotor.getClosedLoopController().setReference(
            position.in(Rotations), // ! shouldn't there be a * PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION here???
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(); // ! what 
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.drivePosition = Rotations.of((driveEncoder.getPosition() // ! try to understand the coupling stuff colin mentioned
            + turnRelativeEncoder.getPosition() / PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION * PhysicalConstants.COUPLING_RATIO)
            / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION
        ).in(Radians);
        inputs.driveVelocity = Rotations.per(Minute).of(driveEncoder.getVelocity() / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION).in(RadiansPerSecond);
        inputs.driveTemperature = driveMotor.getMotorTemperature();
        
        inputs.turnVoltage = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage(); // ! what
        inputs.turnCurrent = turnMotor.getOutputCurrent();
        inputs.turnPosition = turnAbsoluteEncoder.getPosition() / PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION;
        inputs.turnTemperature = turnMotor.getMotorTemperature();
    }
}