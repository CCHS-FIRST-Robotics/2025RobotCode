package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.*;
import edu.wpi.first.units.measure.*;

public class AlgaIOTalonFX implements AlgaIO {
    private final TalonFX motor;

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public AlgaIOTalonFX(int id) {
        motor = new TalonFX(id);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue().in(Volts);
        inputs.motorCurrent = currentSignal.getValue().in(Amps);
        inputs.motorVelocity = velocitySignal.getValue().in(RotationsPerSecond); // ! might wanna change units later
        inputs.motorTemperature = temperatureSignal.getValue().in(Celsius);
    }
}