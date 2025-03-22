package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.*;

public class CoralIOReal implements CoralIO{
    private final TalonFX elevatorMotor;
    private final TalonFX armMotor;

    private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    private final CANcoder elevatorCancoder;
    private final CANcoderConfiguration elevatorCancoderConfig  = new CANcoderConfiguration();
    private final Slot0Configs elevatorPIDF = elevatorConfig.Slot0;
    private final MotionMagicConfigs elevatorMotionMagicConfig = elevatorConfig.MotionMagic;
    private final MotionMagicVoltage elevatorMotionMagicVoltage = new MotionMagicVoltage(0);

    private double kPElevator = 20;
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

    private double kPArm = 40;
    private double kIArm = 0;
    private double kDArm = 0; 
    private double kGArm = 0.65;
    private double kSArm = 0;
    private double kVArm = 0.1121914734;
    private double kAArm = 0;

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

    CoralIOInputs inputs = new CoralIOInputs();
    
    public CoralIOReal(
        int elevatorId, 
        int elevatorCancoderId, 
        int armId, 
        int armCancoderId
    ) {
        elevatorMotor = new TalonFX(elevatorId); // Falcon500
        armMotor = new TalonFX(armId); // Falcon500

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
        elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 1.5;
        elevatorMotionMagicConfig.MotionMagicAcceleration = 20;
        elevatorMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        elevatorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 40;
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
        armPIDF.kS = kSArm; // 0 cancoder when it's parallel to the ground
        armPIDF.kV = kVArm;
        armPIDF.kA = kAArm;
        armPIDF.GravityType = GravityTypeValue.Arm_Cosine;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 0.2; 
        armMotionMagicConfig.MotionMagicAcceleration = 0.4;
        armMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        armConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armConfig.CurrentLimits.StatorCurrentLimit = 40;
        armMotor.getConfigurator().apply(armConfig);

        // ————— logging ————— //

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
    }

    // ————— elevator ————— //

    @Override
    public void setElevatorVoltage(Voltage volts) {
        elevatorMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setElevatorPosition(Angle position) {
        elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position).withSlot(0));
    }

    @Override
    public boolean elevatorAtSetpoint(Angle position) {
        return Math.abs(position.in(Rotations) - inputs.elevatorEncoderPosition) < 0.1; // ! should figure out if it makes sense to retune these
    }

    // ————— arm ————— //

    @Override
    public void setArmVoltage(Voltage volts) {
        armMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setArmPosition(Angle position) {
        armMotor.setControl(armMotionMagicVoltage.withPosition(position).withSlot(0));
    }
    
    @Override
    public boolean armAtSetpoint(Angle position) {
        return Math.abs(position.in(Rotations) - inputs.armEncoderPosition) < 0.008; // ! might be too exact
    }

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
        inputs.elevatorMotorPosition = positionSignalElevator.getValue().in(Rotations) * PhysicalConstants.ELEVATOR_GEAR_REDUCTION;
        inputs.elevatorMotorVelocity = velocitySignalElevator.getValue().in(RotationsPerSecond) * PhysicalConstants.ELEVATOR_GEAR_REDUCTION;
        inputs.elevatorEncoderPosition = positionAbsoluteSignalElevator.getValue().in(Rotations);
        inputs.elevatorEncoderVelocity = velocityAbsoluteSignalElevator.getValue().in(RotationsPerSecond);
        inputs.elevatorTemperature = temperatureSignalElevator.getValue().in(Celsius);

        inputs.armVoltage = voltageSignalArm.getValue().in(Volts);
        inputs.armCurrent = currentSignalArm.getValue().in(Amps);
        inputs.armMotorPosition = positionSignalArm.getValue().in(Rotations) * PhysicalConstants.ARM_GEAR_REDUCTION;
        inputs.armMotorVelocity = velocitySignalArm.getValue().in(RotationsPerSecond) * PhysicalConstants.ARM_GEAR_REDUCTION;
        inputs.armEncoderPosition = positionAbsoluteSignalArm.getValue().in(Rotations);
        inputs.armEncoderVelocity = velocityAbsoluteSignalArm.getValue().in(RotationsPerSecond);
        inputs.armTemperature = temperatureSignalArm.getValue().in(Celsius);

        this.inputs = inputs;
    }
}