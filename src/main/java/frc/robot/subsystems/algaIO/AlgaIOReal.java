package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.*;

public class AlgaIOReal implements AlgaIO {
    private final TalonFX algaMotor;
    private final TalonFXConfiguration algaConfig = new TalonFXConfiguration();
    private final TalonFX drawBridgeMotor;
    private final TalonFXConfiguration drawBridgeConfig = new TalonFXConfiguration();

    private final StatusSignal<Voltage> algaVoltageSignal;
    private final StatusSignal<Current> algaCurrentSignal;
    private final StatusSignal<Temperature> algaTemperatureSignal;

    private final StatusSignal<Voltage> drawbridgeVoltageSignal;
    private final StatusSignal<Current> drawbridgeCurrentSignal;
    private final StatusSignal<Temperature> drawbridgeTemperatureSignal;

    public AlgaIOReal(int algaId, int drawbridgeId) {
        algaMotor = new TalonFX(algaId);
        drawBridgeMotor = new TalonFX(drawbridgeId);

        algaConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        algaMotor.getConfigurator().apply(algaConfig);

        drawBridgeConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(30));
        drawBridgeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        drawBridgeMotor.getConfigurator().apply(drawBridgeConfig);

        algaVoltageSignal = algaMotor.getMotorVoltage();
        algaCurrentSignal = algaMotor.getStatorCurrent();
        algaTemperatureSignal = algaMotor.getDeviceTemp();

        drawbridgeVoltageSignal = drawBridgeMotor.getMotorVoltage();
        drawbridgeCurrentSignal = drawBridgeMotor.getStatorCurrent();
        drawbridgeTemperatureSignal = drawBridgeMotor.getDeviceTemp();
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
        BaseStatusSignal.refreshAll(
            algaVoltageSignal,
            algaCurrentSignal,
            algaTemperatureSignal,

            drawbridgeVoltageSignal,
            drawbridgeCurrentSignal,
            drawbridgeTemperatureSignal
        );

        inputs.algaVoltage = algaVoltageSignal.getValue().in(Volts);
        inputs.algaCurrent = algaCurrentSignal.getValue().in(Amps);
        inputs.algaTemperature = algaTemperatureSignal.getValue().in(Celsius);

        inputs.drawbridgeVoltage = drawbridgeVoltageSignal.getValue().in(Volts);
        inputs.drawbridgeCurrent = drawbridgeCurrentSignal.getValue().in(Amps);
        inputs.drawbridgeTemperature = drawbridgeTemperatureSignal.getValue().in(Celsius);
    }
}