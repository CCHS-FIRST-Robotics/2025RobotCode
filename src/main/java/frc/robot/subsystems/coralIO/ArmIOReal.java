package frc.robot.subsystems.coralIO;

import static edu.wpi.first.units.Units.*;
    
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.*;

public class ArmIOReal implements ArmIO{
    private final TalonFX armMotor;
    private final SparkMax wristMotor;
    private final SparkMax clawMotor;

    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
    private final Slot0Configs armPIDF = armConfig.Slot0;
    private final MotionMagicConfigs armMotionMagicConfig = armConfig.MotionMagic;
    private final MotionMagicVoltage armMotionMagicVoltage = new MotionMagicVoltage(0);

    private double kPArm = 0;
    private double kIArm = 0;
    private double kDArm = 0;
    private double kSArm = 0;
    private double kVArm = 0;
    private double kAArm = 0;

    private final SparkMaxConfig wristConfig = new SparkMaxConfig();
    private final RelativeEncoder wristEncoder;

    private double kPWrist = 0;
    private double kIWrist = 0;
    private double kDWrist = 0;

    private final SparkMaxConfig clawConfig = new SparkMaxConfig();
    private final RelativeEncoder clawEncoder;

    private double kPClaw = 0;
    private double kIClaw = 0;
    private double kDClaw = 0;

    private StatusSignal<Voltage> voltageSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Temperature> temperatureSignal;
    
    public ArmIOReal(int armId, int wristId, int clawId) {
        armMotor = new TalonFX(armId);
        wristMotor = new SparkMax(wristId, MotorType.kBrushed);
        clawMotor = new SparkMax(clawId, MotorType.kBrushed);

        armPIDF.kP = kPArm;
        armPIDF.kI = kIArm;
        armPIDF.kD = kDArm;
        armPIDF.kS = kSArm;
        armPIDF.kV = kVArm;
        armPIDF.kA = kAArm;
        armMotionMagicConfig.MotionMagicCruiseVelocity = 100; // motor max rps // ! idk if this is correct
        armMotionMagicConfig.MotionMagicAcceleration = 1;
        armMotionMagicConfig.MotionMagicJerk = 1;
        armMotor.getConfigurator().apply(armConfig);

        wristMotor.setCANTimeout(500);
        wristEncoder = wristMotor.getEncoder();
        wristEncoder.setPosition(0);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPWrist, kIWrist, kDWrist, ClosedLoopSlot.kSlot0);
        wristConfig.signals.primaryEncoderPositionPeriodMs(20);
        wristConfig.encoder.quadratureAverageDepth(2);
        wristConfig.smartCurrentLimit(30);
        wristConfig.voltageCompensation(12);
        wristConfig.idleMode(IdleMode.kBrake);
        wristMotor.setCANTimeout(0);
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        clawMotor.setCANTimeout(500);
        clawEncoder = wristMotor.getEncoder();
        clawEncoder.setPosition(0);
        clawConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPClaw, kIClaw, kDClaw, ClosedLoopSlot.kSlot0);
        clawConfig.signals.primaryEncoderPositionPeriodMs(20);
        clawConfig.encoder.quadratureAverageDepth(2);
        clawConfig.smartCurrentLimit(30);
        clawConfig.voltageCompensation(12);
        clawConfig.idleMode(IdleMode.kBrake);
        clawMotor.setCANTimeout(0);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        voltageSignal = armMotor.getMotorVoltage();
        currentSignal = armMotor.getStatorCurrent();
        positionSignal = armMotor.getPosition();
        velocitySignal = armMotor.getVelocity();
        temperatureSignal = armMotor.getDeviceTemp();
    }

    public void setArmPosition(Angle position){
        armMotor.setControl(armMotionMagicVoltage.withPosition(position.in(Rotations)).withSlot(0));
    }

    public void setWristPosition(Angle position){ // probably only two states
        wristMotor.getClosedLoopController().setReference(
            position.in(Rotations),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    public void setClawPosition(Angle position){ // open or closed
        wristMotor.getClosedLoopController().setReference(
            position.in(Rotations),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, positionSignal, velocitySignal, temperatureSignal);

        inputs.armVoltage = voltageSignal.getValue().in(Volts);
        inputs.armCurrent = currentSignal.getValue().in(Amps);
        inputs.armPosition = positionSignal.getValue().in(Rotations); // ! might wanna change units later
        inputs.armVelocity = velocitySignal.getValue().in(RotationsPerSecond);
        inputs.armTemperature = temperatureSignal.getValue().in(Celsius);

        inputs.wristCurrent = wristMotor.getOutputCurrent();
        inputs.wristVoltage = wristMotor.getBusVoltage();
        inputs.wristPosition = wristEncoder.getPosition(); // ! units
        inputs.wristVelocity = wristEncoder.getVelocity();
        inputs.wristTemperature = wristMotor.getMotorTemperature();

        inputs.clawCurrent = clawMotor.getOutputCurrent();
        inputs.clawVoltage = clawMotor.getBusVoltage();
        inputs.clawPosition = clawEncoder.getPosition(); // ! units
        inputs.clawVelocity = clawEncoder.getVelocity();
        inputs.clawTemperature = clawMotor.getMotorTemperature();
    }
}