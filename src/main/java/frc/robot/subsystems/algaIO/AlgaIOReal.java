package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;

public class AlgaIOReal implements AlgaIO {
    private final TalonFX algaMotor;
    private final TalonFXConfiguration algaConfig = new TalonFXConfiguration();
    private final SparkMax drawBridgeMotor;
    private final SparkMaxConfig drawBridgeConfig = new SparkMaxConfig();

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public AlgaIOReal(int algaId, int drawbridgeId) {
        algaMotor = new TalonFX(algaId);
        drawBridgeMotor = new SparkMax(drawbridgeId, MotorType.kBrushless);

        algaConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); // !
        algaMotor.getConfigurator().apply(algaConfig);

        drawBridgeMotor.setCANTimeout(500);
        drawBridgeConfig.smartCurrentLimit(30);
        drawBridgeConfig.voltageCompensation(12);
        drawBridgeConfig.inverted(true); // ! 
        drawBridgeConfig.idleMode(IdleMode.kBrake); // ! 
        drawBridgeMotor.setCANTimeout(0);
        drawBridgeMotor.configure(drawBridgeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        voltageSignal = algaMotor.getMotorVoltage();
        currentSignal = algaMotor.getStatorCurrent();
        temperatureSignal = algaMotor.getDeviceTemp();
    }

    @Override
    public void setAlgaVoltage(Voltage volts) {
        algaMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setDrawbridgeVoltage(Voltage volts){
        drawBridgeMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, temperatureSignal);

        inputs.algaVoltage = voltageSignal.getValue().in(Volts);
        inputs.algaCurrent = currentSignal.getValue().in(Amps);
        inputs.algaTemperature = temperatureSignal.getValue().in(Celsius);

        inputs.drawbridgeVoltage = drawBridgeMotor.getAppliedOutput() * drawBridgeMotor.getBusVoltage();
        inputs.drawbridgeCurrent = drawBridgeMotor.getOutputCurrent();
        inputs.drawbridgeTemperature = drawBridgeMotor.getMotorTemperature();
    }
}