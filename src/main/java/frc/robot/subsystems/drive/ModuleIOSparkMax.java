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
    
    public final int index;

    private double driveKp = 0.00001;
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = 0.136898;
    private double driveKa = 0.020864;

    private double turnKp = 0.1;
    private double turnKi = 0;
    private double turnKd = 0;

    // ! check colin's math!
    // ! move these to hardwareConstants


    private AngularVelocity prevDriveVelocity = RadiansPerSecond.of(0.0);

    // ! probably scuffed in translation
    public ModuleIOSparkMax(int index) {
        driveMotor = new SparkMax(10 * index + 1, MotorType.kBrushless);
        turnMotor = new SparkMax(10 * index + 2, MotorType.kBrushless);
        this.index = index;

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
        driveConfig.signals.primaryEncoderPositionPeriodMs(10);
        turnConfig.signals.primaryEncoderVelocityPeriodMs(20); // ! maybe wrong
        
        driveConfig.encoder.quadratureMeasurementPeriod(10); // ! could be not quadrature
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
        driveMotor.getClosedLoopController().setReference( // ! probably scuffed in translation
            velocity.in(Rotations.per(Minute)) * PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION,
            SparkMax.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            driveFeedforward.calculateWithVelocities(
                prevDriveVelocity.in(RotationsPerSecond),
                velocity.in(RotationsPerSecond)
            )
        );

        prevDriveVelocity = velocity;
    }

    @Override
    public void setTurnPosition(Angle position) {
        // adjust from [-PI, PI] to [0, 2PI]  // ! out of curiosity, why
        position = Rotations.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI)); 
        
        turnMotor.getClosedLoopController().setReference(
            position.in(Rotations), 
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(); // ! what 
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.drivePosition = // ! what the fuck
            (driveEncoder.getPosition()
            + turnRelativeEncoder.getPosition() / PhysicalConstants.TURN_AFTER_ENCODER_REDUCTION * PhysicalConstants.COUPLING_RATIO)
            / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION;
        inputs.driveVelocity = Rotations.per(Minute).of(driveEncoder.getVelocity() / PhysicalConstants.DRIVE_AFTER_ENCODER_REDUCTION).in(RotationsPerSecond);
        inputs.driveTemperature = driveMotor.getMotorTemperature();
        
        inputs.turnVoltage = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage(); // ! what
        inputs.turnCurrent = turnMotor.getOutputCurrent();
        inputs.turnPosition = turnAbsoluteEncoder.getPosition();
        inputs.turnTemperature = turnMotor.getMotorTemperature();
    }
}