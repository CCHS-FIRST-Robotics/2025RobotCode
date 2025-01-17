package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOSparkMax implements ModuleIO {
    // ! change to d and t lol
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    private final SparkMaxConfig turnConfig = new SparkMaxConfig();
    private final SimpleMotorFeedforward driveFeedforward;

    private final RelativeEncoder driveEncoder; // NEO
    private final RelativeEncoder turnRelativeEncoder; // NEO
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder
    
    int index;

    private double driveKp = 0.00001;
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = 0.136898;
    private double driveKa = 0.020864;

    private double turnKp = 8;
    private double turnKd = 0;

    // ! check colin's math! 
    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;
    private final double couplingRatio = 50.0 / 14.0;

    private AngularVelocity prevDriveVelocity = RadiansPerSecond.of(0.0);

    // ! probably scuffed in translation
    public ModuleIOSparkMax(int index) {
        driveMotor = new SparkMax(2 + 2 * index, MotorType.kBrushless);
        turnMotor = new SparkMax(1 + 2 * index, MotorType.kBrushless);
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
        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(turnKp, 0, turnKd, ClosedLoopSlot.kSlot0);
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
    
    public void setDriveVoltage(Voltage volts) {
        driveMotor.setVoltage(volts.in(Volts));
    }
    
    public void setTurnVoltage(Voltage volts) {
        turnMotor.setVoltage(volts.in(Volts));
    }
    
    public void setDriveVelocity(AngularVelocity velocity) {
        driveMotor.getClosedLoopController().setReference( // ! probably scuffed in translation
            velocity.in(Rotations.per(Minute)) * driveAfterEncoderReduction,
            SparkMax.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            driveFeedforward.calculateWithVelocities(
                prevDriveVelocity.in(RadiansPerSecond),
                velocity.in(RadiansPerSecond)
            )
        );

        prevDriveVelocity = velocity;
    }

    public void setTurnPosition(Angle position) {
        // Adjust from [-PI, PI] -> [0, 2PI]
        position = Radians.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));
            
        turnMotor.getClosedLoopController().setReference(
            position.in(Rotations),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    // ! idk if these are necessary
    public void setDriveBrakeMode(boolean enable) {
        driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
    
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveRawPositionRad = // doesnt account for coupling
            Rotations.of(driveEncoder.getPosition() / driveAfterEncoderReduction
        );
        inputs.drivePositionRad = 
            Rotations.of((driveEncoder.getPosition()
            + turnRelativeEncoder.getPosition() / turnAfterEncoderReduction * couplingRatio)
            / driveAfterEncoderReduction
        );
        inputs.driveVelocityRadPerSec = 
            Rotations.per(Minute).of(driveEncoder.getVelocity()
            / driveAfterEncoderReduction
        );
        inputs.driveAppliedVolts = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
        inputs.driveCurrentAmps = Amps.of(driveMotor.getOutputCurrent());
        inputs.driveTempCelcius = Celsius.of(driveMotor.getMotorTemperature());

        inputs.turnAbsolutePositionRad = Radians.of(
                MathUtil.angleModulus(
                        new Rotation2d(
                                turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                                        * 2 * Math.PI)
                                .getRadians()));

        inputs.turnPositionRad = Rotations.of(turnRelativeEncoder.getPosition()
                / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = Rotations.per(Minute).of(turnRelativeEncoder.getVelocity()
                / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = Volts.of(turnMotor.getAppliedOutput() * turnMotor.getBusVoltage());
        inputs.turnCurrentAmps = Amps.of(turnMotor.getOutputCurrent());
        inputs.turnTempCelcius = Celsius.of(turnMotor.getMotorTemperature());
    }
}