package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.*;

public class CoralIOReal implements CoralIO{
    private final TalonFX elevatorMotor;
    private final TalonFX armMotor;
    private final SparkMax wristMotor;
    private final SparkMax clawMotor;

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

    private double kPArm = 20; // ! sysid all gains
    private double kIArm = 0;
    private double kDArm = 0;
    private double kGArm = 0.15;
    private double kSArm = 0;
    private double kVArm = 0.1121914734;
    private double kAArm = 0;

    private final SparkMaxConfig wristConfig = new SparkMaxConfig();
    private final CANcoder wristCancoder;
    private final CANcoderConfiguration wristCancoderConfig = new CANcoderConfiguration();
    private final PIDController wristPID;
    private final SimpleMotorFeedforward wristFF;
    private final TrapezoidProfile wristProfile;

    private double kPWrist = 1; // ! sysid all gains
    private double kIWrist = 0;
    private double kDWrist = 0;
    private double kGWrist = 0;
    private double kSWrist = 0.35284;
    private double kVWrist = 0.058472;
    private double kAWrist = 0.0039432;

    private final SparkMaxConfig clawConfig = new SparkMaxConfig();
    private final SparkAnalogSensor clawSwitch;

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
        clawMotor = new SparkMax(clawId, MotorType.kBrushed); // BAG

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
        elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 1; // max is 1
        elevatorMotionMagicConfig.MotionMagicAcceleration = 1;
        elevatorMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        elevatorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 30;
        elevatorMotor.getConfigurator().apply(elevatorConfig);
    
        // ————— arm ————— //

        // encoder
        armCancoder = new CANcoder(armCancoderId);
        armCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
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
        armMotionMagicConfig.MotionMagicCruiseVelocity = 0.05; // ! make this higher hopefully
        armMotionMagicConfig.MotionMagicAcceleration = 1;
        armMotionMagicConfig.MotionMagicJerk = 1;
        // misc
        armConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armConfig.CurrentLimits.StatorCurrentLimit = 30;
        armMotor.getConfigurator().apply(armConfig);

        // ————— wrist ————— //

        // encoder
        wristCancoder = new CANcoder(wristCancoderId);
        wristCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        wristCancoderConfig.MagnetSensor.MagnetOffset = PhysicalConstants.WRIST_ENCODER_OFFSET.in(Rotations); 
        wristCancoder.getConfigurator().apply(wristCancoderConfig);
        // pid
        wristPID = new PIDController(kPWrist, kIWrist, kDWrist);
        wristFF = new SimpleMotorFeedforward(kSWrist, kVWrist, kAWrist);
        wristProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.01, 10));
        // misc
        wristMotor.setCANTimeout(500);
        wristConfig.smartCurrentLimit(20);
        wristConfig.voltageCompensation(12);
        wristConfig.idleMode(IdleMode.kBrake);
        wristMotor.setCANTimeout(0);
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ————— claw ————— //

        // limit switch
        clawSwitch = clawMotor.getAnalog();
        // misc
        clawMotor.setCANTimeout(500);
        clawConfig.smartCurrentLimit(20);
        clawConfig.voltageCompensation(12);
        clawConfig.idleMode(IdleMode.kBrake);
        clawMotor.setCANTimeout(0);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        wristMotor.setVoltage(volts);
    }

    @Override
    public void setWristPosition(Angle position){
        // TrapezoidProfile.State targetState = wristProfile.calculate(
        //     VirtualConstants.PERIOD, 
        //     new TrapezoidProfile.State(inputs.wristAbsolutePosition, 0), 
        //     new TrapezoidProfile.State(position.in(Rotations), 0)
        // );

        this.setWristVoltage(
            Volts.of(
                wristPID.calculate(inputs.wristPosition, position.in(Rotations))
                // + kGWrist
                // + wristFF.calculate(targetState.velocity)
            )
        );
    }
    
    // ————— claw ————— //

    @Override
    public void setClawVoltage(Voltage volts){
        clawMotor.setVoltage(volts);
    }

    @Override 
    public void setClawPosition(boolean open){ // 0 normally, 3.3 when switch on
        if (open) { // open claw
            if(inputs.clawSwitch < 2.5){ // if switch not on
                this.setClawVoltage(Volts.of(-1));
            }else{
                this.setClawVoltage(Volts.of(1)); // ! idk if this actually does anything
            }
        } else { // close claw
            this.setClawVoltage(Volts.of(1));
        }
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
            temperatureSignalArm,

            positionAbsoluteSignalWrist,
            velocityAbsoluteSignalWrist
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
        inputs.armPosition = positionSignalArm.getValue().in(Rotations) * PhysicalConstants.ARM_GEAR_REDUCTION;
        inputs.armVelocity = velocitySignalArm.getValue().in(RotationsPerSecond) * PhysicalConstants.ARM_GEAR_REDUCTION;
        inputs.armAbsolutePosition = positionAbsoluteSignalArm.getValue().in(Rotations);
        inputs.armAbsoluteVelocity = velocityAbsoluteSignalArm.getValue().in(RotationsPerSecond);
        inputs.armTemperature = temperatureSignalArm.getValue().in(Celsius);

        inputs.wristVoltage = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
        inputs.wristCurrent = wristMotor.getOutputCurrent();
        inputs.wristPosition = positionAbsoluteSignalWrist.getValue().in(Rotations) * PhysicalConstants.WRIST_GEAR_REDUCTION;
        inputs.wristVelocity = velocityAbsoluteSignalWrist.getValue().in(RotationsPerSecond) * PhysicalConstants.WRIST_GEAR_REDUCTION;
        inputs.wristAbsolutePosition = positionAbsoluteSignalWrist.getValue().in(Rotations);
        inputs.wristAbsoluteVelocity = velocityAbsoluteSignalWrist.getValue().in(RotationsPerSecond);
        inputs.wristTemperature = wristMotor.getMotorTemperature();

        inputs.clawVoltage = clawMotor.getAppliedOutput() * clawMotor.getBusVoltage();
        inputs.clawCurrent = clawMotor.getOutputCurrent();
        inputs.clawSwitch = clawSwitch.getPosition();
        inputs.clawTemperature = clawMotor.getMotorTemperature();

        this.inputs = inputs;
    }
}