package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PhysicalConstants;

public class CoralIOReal implements CoralIO{
    private final TalonFX elevatorMotor;
    private final TalonFX armMotor;
    private final SparkMax wristMotor;

    private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    private final CANcoder elevatorCancoder;
    private final CANcoderConfiguration elevatorCancoderConfig  = new CANcoderConfiguration();
    private final Slot0Configs elevatorPIDF = elevatorConfig.Slot0;
    private final MotionMagicConfigs elevatorMotionMagicConfig = elevatorConfig.MotionMagic;
    private final MotionMagicVoltage elevatorMotionMagicVoltage = new MotionMagicVoltage(0);

    private double kPElevator = 20; // ! sysid all gains
    private double kIElevator = 0;
    private double kDElevator = 0;
    private double kGElevator = 0;
    private double kSElevator = 0;
    private double kVElevator = 0.1121914734;
    private double kAElevator = 0;

    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
    private final CANcoder armCancoder;
    private final CANcoderConfiguration armCancoderConfig = new CANcoderConfiguration(); 
    private final Slot0Configs armPIDF = armConfig.Slot0;
    private final MotionMagicConfigs armMotionMagicConfig = armConfig.MotionMagic;
    private final MotionMagicVoltage armMotionMagicVoltage = new MotionMagicVoltage(0);

    private double kPArm = 20;
    private double kIArm = 0;
    private double kDArm = 0;
    private double kGArm = 0.15;
    private double kSArm = 0;
    private double kVArm = 0.1121914734;
    private double kAArm = 0;

    private final SparkMaxConfig wristConfig = new SparkMaxConfig(); // ! maybe consider using TalonSRX
    private final CANcoder wristCancoder;
    private final CANcoderConfiguration wristCancoderConfig = new CANcoderConfiguration();
    private final PIDController wristPID;

    private double kPWrist = 1;
    private double kIWrist = 0;
    private double kDWrist = 0;
    private double kGWrist = 0;

    private final StatusSignal<Voltage> voltageSignalElevator;
    private final StatusSignal<Current> currentSignalElevator;
    private final StatusSignal<Angle> positionSignalElevator;
    private final StatusSignal<AngularVelocity> velocitySignalElevator;
    private final StatusSignal<Angle> positionAbsoluteSignalElevator;
    private final StatusSignal<AngularVelocity> velocityAbsoluteSignalElevator;
    private final StatusSignal<Temperature> temperatureSignalElevator;

    private final StatusSignal<Voltage> voltageSignalArm;
    private final StatusSignal<Current> currentSignalArm;
    private final StatusSignal<Angle> positionSignalArm;
    private final StatusSignal<AngularVelocity> velocitySignalArm;
    private final StatusSignal<Angle> positionAbsoluteSignalArm;
    private final StatusSignal<AngularVelocity> velocityAbsoluteSignalArm;
    private final StatusSignal<Temperature> temperatureSignalArm;

    private final StatusSignal<Angle> positionAbsoluteSignalWrist;
    private final StatusSignal<AngularVelocity> velocityAbsoluteSignalWrist;

    private CoralIOInputs inputs = new CoralIOInputs();
    
    public CoralIOReal(
        int elevatorId, 
        int elevatorCancoderId, 
        int armId, 
        int armCancoderId, 
        int wristId, 
        int wristCancoderId, 
        int clawId
    ) {
        elevatorMotor = new TalonFX(elevatorId); // Falcon500
        armMotor = new TalonFX(armId); // Falcon500
        wristMotor = new SparkMax(wristId, MotorType.kBrushed); // RedLine

        // ————— elevator ————— //

        // encoder
        elevatorCancoder = new CANcoder(elevatorCancoderId);
        elevatorCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        elevatorCancoderConfig.MagnetSensor.MagnetOffset = PhysicalConstants.ELEVATOR_ENCODER_OFFSET.in(Rotations);
        elevatorCancoder.getConfigurator().apply(elevatorCancoderConfig);
        elevatorConfig.Feedback.withRemoteCANcoder(elevatorCancoder);
        // pid
        elevatorPIDF.kP = kPElevator;
        elevatorPIDF.kI = kIElevator;
        elevatorPIDF.kD = kDElevator;
        elevatorPIDF.kG = kGElevator;
        elevatorPIDF.kS = kSElevator;
        elevatorPIDF.kV = kVElevator;
        elevatorPIDF.kA = kAElevator;
        elevatorPIDF.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 100;
        elevatorMotionMagicConfig.MotionMagicAcceleration = 1;
        elevatorMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        elevatorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        elevatorMotor.getConfigurator().apply(elevatorConfig);
    
        // ————— arm ————— //

        // encoder
        armCancoder = new CANcoder(armCancoderId);
        armCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armCancoderConfig.MagnetSensor.MagnetOffset = PhysicalConstants.ARM_ENCODER_OFFSET.in(Rotations); 
        armCancoder.getConfigurator().apply(armCancoderConfig);
        armConfig.Feedback.withRemoteCANcoder(armCancoder);
        // pid
        armPIDF.kP = kPArm;
        armPIDF.kI = kIArm;
        armPIDF.kD = kDArm;
        armPIDF.kG = kGArm;
        armPIDF.kS = kSArm;
        armPIDF.kV = kVArm;
        armPIDF.kA = kAArm;
        armPIDF.GravityType = GravityTypeValue.Arm_Cosine;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 0.5;
        armMotionMagicConfig.MotionMagicAcceleration = 1;
        armMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        armConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        armMotor.getConfigurator().apply(armConfig);

        // ————— wrist ————— //

        // encoder
        wristCancoder = new CANcoder(wristCancoderId);
        wristCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // ! figure
        wristCancoderConfig.MagnetSensor.MagnetOffset = PhysicalConstants.WRIST_ENCODER_OFFSET.in(Rotations); 
        wristCancoder.getConfigurator().apply(wristCancoderConfig);
        // pid
        wristPID = new PIDController(kPWrist, kIWrist, kDWrist);
        // misc
        wristMotor.setCANTimeout(500);
        wristConfig.signals.primaryEncoderPositionPeriodMs(20);
        wristConfig.encoder.quadratureAverageDepth(2);
        wristConfig.smartCurrentLimit(30);
        wristConfig.voltageCompensation(12);
        wristConfig.idleMode(IdleMode.kBrake);
        wristMotor.setCANTimeout(0); // ! try with resetsafeparameters
        System.out.println("HLIUHWALKJJFLKJFLKJWAFA" + wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // ————— misc ————— //

        voltageSignalElevator = elevatorMotor.getMotorVoltage();
        currentSignalElevator = elevatorMotor.getStatorCurrent();
        positionSignalElevator = elevatorMotor.getPosition();
        velocitySignalElevator = elevatorMotor.getVelocity();
        positionAbsoluteSignalElevator = elevatorCancoder.getPosition();
        velocityAbsoluteSignalElevator = elevatorCancoder.getVelocity();
        temperatureSignalElevator = elevatorMotor.getDeviceTemp();

        voltageSignalArm = armMotor.getMotorVoltage();
        currentSignalArm = armMotor.getStatorCurrent();
        positionSignalArm = armMotor.getPosition();
        velocitySignalArm = armMotor.getVelocity();
        positionAbsoluteSignalArm = armCancoder.getPosition();
        velocityAbsoluteSignalArm = armCancoder.getVelocity();
        temperatureSignalArm = armMotor.getDeviceTemp();

        positionAbsoluteSignalWrist = wristCancoder.getPosition();
        velocityAbsoluteSignalWrist = wristCancoder.getVelocity();
    }

    // ————— elevator ————— //

    @Override
    public void setElevatorVoltage(Voltage volts){
        elevatorMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setElevatorPosition(Angle position){
        elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position).withSlot(0));
    }

    // ————— arm ————— //

    @Override
    public void setArmVoltage(Voltage volts){
        armMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setArmPosition(Angle position){
        armMotor.setControl(armMotionMagicVoltage.withPosition(position).withSlot(0));
    }

    // ————— wrist ————— //

    @Override
    public void setWristVoltage(Voltage volts){
        wristMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setWristPosition(Angle position){
        this.setWristVoltage(Volts.of(
            wristPID.calculate(inputs.wristPosition, position.in(Rotations))
            + kGWrist
        ));
    }
    
    // ————— claw ————— //

    // @Override
    // public void setClawVoltage(Voltage volts){}

    // @Override
    // public void setClawPosition(boolean open){}

    // ————— logging ————— //

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            voltageSignalElevator,
            currentSignalElevator,
            positionSignalElevator,
            velocitySignalElevator,
            positionAbsoluteSignalElevator,
            velocityAbsoluteSignalElevator,
            temperatureSignalElevator,

            voltageSignalArm, 
            currentSignalArm, 
            positionSignalArm, 
            velocitySignalArm, 
            positionAbsoluteSignalArm,
            velocityAbsoluteSignalArm,
            temperatureSignalArm
        );

        inputs.elevatorVoltage = voltageSignalElevator.getValue().in(Volts);
        inputs.elevatorCurrent = currentSignalElevator.getValue().in(Amps);
        inputs.elevatorPosition = positionSignalElevator.getValue().in(Rotations);
        inputs.elevatorVelocity = velocitySignalElevator.getValue().in(RotationsPerSecond);
        inputs.elevatorAbsolutePosition = positionAbsoluteSignalElevator.getValue().in(Rotations);
        inputs.elevatorAbsoluteVelocity = velocityAbsoluteSignalElevator.getValue().in(RotationsPerSecond);
        inputs.elevatorTemperature = temperatureSignalElevator.getValue().in(Celsius);

        inputs.armVoltage = voltageSignalArm.getValue().in(Volts);
        inputs.armCurrent = currentSignalArm.getValue().in(Amps);
        inputs.armPosition = positionSignalArm.getValue().in(Rotations);
        inputs.armVelocity = velocitySignalArm.getValue().in(RotationsPerSecond);
        inputs.armAbsolutePosition = positionAbsoluteSignalArm.getValue().in(Rotations);
        inputs.armAbsoluteVelocity = velocityAbsoluteSignalArm.getValue().in(RotationsPerSecond);
        inputs.armTemperature = temperatureSignalArm.getValue().in(Celsius);

        inputs.wristCurrent = wristMotor.getOutputCurrent();
        inputs.wristVoltage = wristMotor.getBusVoltage();
        inputs.wristPosition = positionAbsoluteSignalWrist.getValue().in(Rotations);
        inputs.wristVelocity = velocityAbsoluteSignalWrist.getValue().in(RotationsPerSecond);
        inputs.wristTemperature = wristMotor.getMotorTemperature();

        this.inputs = inputs;
    }
}