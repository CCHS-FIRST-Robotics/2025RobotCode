package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.*;

// ! look over william's code
public class AlgalOSparkMax implements AlgaIO {
    private final SparkMax motor;

    private final RelativeEncoder encoder;

    // private final StatusSignal<Voltage> voltageSignal;
    // private final StatusSignal<Current> currentSignal;
    // private final StatusSignal<AngularVelocity> velocitySignal;
    // private final StatusSignal<Temperature> temperatureSignal;

    public AlgalOSparkMax(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // voltageSignal = motor.getMotorVoltage();
        // currentSignal = motor.getStatorCurrent();
        // velocitySignal = motor.getVelocity();
        // temperatureSignal = motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        //BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorVelocity = encoder.getVelocity() / 60;
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}