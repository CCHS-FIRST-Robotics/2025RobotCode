package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.*;
import edu.wpi.first.units.measure.*;

public class AlgaIOReal implements AlgaIO {
    private final TalonFX motor;

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public AlgaIOReal(int id) {
        motor = new TalonFX(id);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        temperatureSignal = motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue().in(Volts);
        inputs.motorCurrent = currentSignal.getValue().in(Amps);
        inputs.motorTemperature = temperatureSignal.getValue().in(Celsius);
    }
}