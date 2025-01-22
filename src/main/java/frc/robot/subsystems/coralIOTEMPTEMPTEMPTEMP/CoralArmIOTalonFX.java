package frc.robot.subsystems.coralIOTEMPTEMPTEMPTEMP;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

public class CoralArmIOTalonFX implements CoralArmIO {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final Slot0Configs PIDF = motorConfig.Slot0;
    private final MotionMagicConfigs motionMagicConfig = motorConfig.MotionMagic;
    private final MotionMagicVoltage motorMotionMagicVoltage = new MotionMagicVoltage(0);

    // ! absolute encoder

    private final double kP = 20; // ! tuning lmao
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0;
    private final double kV = 0;
    private final double kA = 0;
    // ! kg is a thing lmao

    private StatusSignal<Voltage> voltageSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Temperature> temperatureSignal;
    
    public CoralArmIOTalonFX(int id) {
        motor = new TalonFX(id);

        PIDF.kP = kP;
        PIDF.kI = kI;
        PIDF.kD = kD;
        PIDF.kS = kS;
        PIDF.kV = kV;
        PIDF.kA = kA;

        motionMagicConfig.MotionMagicCruiseVelocity = 100; // motor max rps // ! idk if this is correct
        motionMagicConfig.MotionMagicAcceleration = 1;
        motionMagicConfig.MotionMagicJerk = 1;

        motor.getConfigurator().apply(motorConfig);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setPosition(Angle position){
        motor.setControl(motorMotionMagicVoltage.withPosition(position.in(Rotations)).withSlot(0));
    }

    @Override
    public void updateInputs(CoralArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, positionSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue().in(Volts);
        inputs.motorCurrent = currentSignal.getValue().in(Amps);
        inputs.motorPosition = positionSignal.getValue().in(Rotations); // ! might wanna change units later
        inputs.motorVelocity = velocitySignal.getValue().in(RotationsPerSecond);
        inputs.motorTemperature = temperatureSignal.getValue().in(Celsius);
    }
}