package frc.robot.subsystems.algaeIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.*;
import edu.wpi.first.units.measure.*;


public class AlgaIOTalonFX implements AlgaIO {
    private TalonFX motor;

    private StatusSignal<Voltage> voltageSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Temperature> temperatureSignal;

    public AlgaIOTalonFX(int id) {
        motor = new TalonFX(id);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        currentSignal.setUpdateFrequency(1 / 0.01); // second number is seconds
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();

        // ! might wanna do current limiting or whatever configs later
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, positionSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue().in(Volts);
        inputs.motorCurrent = currentSignal.getValue().in(Amps);
        inputs.motorVelocity = velocitySignal.getValue().in(RotationsPerSecond); // ! might wanna change units later
        inputs.motorTemperature = temperatureSignal.getValue().in(Celsius);
    }
}