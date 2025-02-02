package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;
    
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.*;
// import com.revrobotics.*;
// import com.revrobotics.spark.*;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.units.measure.*;

public class CoralIOReal implements CoralIO{
    private final TalonFX elevatorMotor;
    private final TalonFX armMotor;
    // private final SparkMax wristMotor; 

    private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    private final Slot0Configs elevatorPIDF = elevatorConfig.Slot0;
    private final MotionMagicConfigs elevatorMotionMagicConfig = elevatorConfig.MotionMagic;
    private final MotionMagicVoltage elevatorMotionMagicVoltage = new MotionMagicVoltage(0);
    private final double elevatorGearingReduction = 100;

    private double kPElevator = 2;
    private double kIElevator = 0;
    private double kDElevator = 0;

    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
    private final Slot0Configs armPIDF = armConfig.Slot0;
    private final MotionMagicConfigs armMotionMagicConfig = armConfig.MotionMagic;
    private final MotionMagicVoltage armMotionMagicVoltage = new MotionMagicVoltage(0);
    private final double armGearingReduction = 100;

    private double kPArm = 2;
    private double kIArm = 0;
    private double kDArm = 0;
    // ! needs kG

    // private final SparkMaxConfig wristConfig = new SparkMaxConfig();
    // private final RelativeEncoder wristEncoder;

    // private double kPWrist = 0;
    // private double kIWrist = 0;
    // private double kDWrist = 0;

    private final StatusSignal<Voltage> voltageSignalElevator;
    private final StatusSignal<Current> currentSignalElevator;
    private final StatusSignal<Angle> positionSignalElevator;
    private final StatusSignal<AngularVelocity> velocitySignalElevator;
    private final StatusSignal<Temperature> temperatureSignalElevator;

    private final StatusSignal<Voltage> voltageSignalArm;
    private final StatusSignal<Current> currentSignalArm;
    private final StatusSignal<Angle> positionSignalArm;
    private final StatusSignal<AngularVelocity> velocitySignalArm;
    private final StatusSignal<Temperature> temperatureSignalArm;
    
    public CoralIOReal(int elevatorId, int armId, int wristId, int clawId) {
        elevatorMotor = new TalonFX(elevatorId);
        armMotor = new TalonFX(armId);
        // wristMotor = new SparkMax(wristId, MotorType.kBrushed);

        elevatorPIDF.kP = kPElevator;
        elevatorPIDF.kI = kIElevator;
        elevatorPIDF.kD = kDElevator;
        elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 100;
        elevatorMotionMagicConfig.MotionMagicAcceleration = 1;
        elevatorMotionMagicConfig.MotionMagicJerk = 1;
        elevatorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        elevatorMotor.setPosition(Rotations.of(0)); // ! position isn't reset every time, check if this works
        elevatorConfig.Feedback.withSensorToMechanismRatio(elevatorGearingReduction); // ! check if this is correct
        elevatorMotor.getConfigurator().apply(elevatorConfig);
    
        armPIDF.kP = kPArm;
        armPIDF.kI = kIArm;
        armPIDF.kD = kDArm;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 100;
        armMotionMagicConfig.MotionMagicAcceleration = 1;
        armMotionMagicConfig.MotionMagicJerk = 1;
        armConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        armMotor.setPosition(Rotations.of(0)); // ! position isn't reset every time, check if this works
        armConfig.Feedback.withRotorToSensorRatio(armGearingReduction); // ! actually configure the cancoder
        armMotor.getConfigurator().apply(armConfig);

        // wristMotor.setCANTimeout(500);
        // wristEncoder = wristMotor.getEncoder();
        // wristEncoder.setPosition(0);
        // wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPWrist, kIWrist, kDWrist, ClosedLoopSlot.kSlot0);
        // wristConfig.signals.primaryEncoderPositionPeriodMs(20);
        // wristConfig.encoder.quadratureAverageDepth(2);
        // wristConfig.smartCurrentLimit(30);
        // wristConfig.voltageCompensation(12);
        // wristConfig.idleMode(IdleMode.kBrake);
        // wristMotor.setCANTimeout(0);
        // wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        voltageSignalElevator = elevatorMotor.getMotorVoltage();
        currentSignalElevator = elevatorMotor.getStatorCurrent();
        positionSignalElevator = elevatorMotor.getPosition();
        velocitySignalElevator = elevatorMotor.getVelocity();
        temperatureSignalElevator = elevatorMotor.getDeviceTemp();

        voltageSignalArm = armMotor.getMotorVoltage();
        currentSignalArm = armMotor.getStatorCurrent();
        positionSignalArm = armMotor.getPosition();
        velocitySignalArm = armMotor.getVelocity();
        temperatureSignalArm = armMotor.getDeviceTemp();
    }

    @Override
    public void setElevatorVoltage(Voltage volts){
        elevatorMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setElevatorPosition(Angle position){
        elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position).withSlot(0));
    }

    @Override
    public void setArmVoltage(Voltage volts){
        armMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setArmPosition(Angle position){
        armMotor.setControl(armMotionMagicVoltage.withPosition(position).withSlot(0));
    }

    // @Override
    // public void setWristVoltage(Voltage volts){
    //     wristMotor.setVoltage(volts.in(Volts));
    // }

    // @Override
    // public void setWristPosition(Angle position){ // probably only two states
    //     wristMotor.getClosedLoopController().setReference(
    //         position.in(Rotations),
    //         SparkMax.ControlType.kPosition,
    //         ClosedLoopSlot.kSlot0
    //     );
    // }

    @Override
    public void setClawVoltage(Voltage volts){}

    @Override
    public void setClawPosition(boolean open){}

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            voltageSignalElevator,
            currentSignalElevator,
            positionSignalElevator,
            velocitySignalElevator,
            temperatureSignalElevator,
            voltageSignalArm, 
            currentSignalArm, 
            positionSignalArm, 
            velocitySignalArm, 
            temperatureSignalArm
        );

        inputs.elevatorVoltage = voltageSignalElevator.getValue().in(Volts);
        inputs.elevatorCurrent = currentSignalElevator.getValue().in(Amps);
        inputs.elevatorPosition = positionSignalElevator.getValue().in(Rotations) / 100;
        inputs.elevatorVelocity = velocitySignalElevator.getValue().in(RotationsPerSecond);
        inputs.elevatorTemperature = temperatureSignalElevator.getValue().in(Celsius);

        inputs.armVoltage = voltageSignalArm.getValue().in(Volts);
        inputs.armCurrent = currentSignalArm.getValue().in(Amps);
        inputs.armPosition = positionSignalArm.getValue().in(Rotations);
        inputs.armVelocity = velocitySignalArm.getValue().in(RotationsPerSecond);
        inputs.armTemperature = temperatureSignalArm.getValue().in(Celsius);

        // inputs.wristCurrent = wristMotor.getOutputCurrent();
        // inputs.wristVoltage = wristMotor.getBusVoltage();
        // inputs.wristPosition = wristEncoder.getPosition(); // ! units
        // inputs.wristVelocity = wristEncoder.getVelocity();
        // inputs.wristTemperature = wristMotor.getMotorTemperature();
    }
}