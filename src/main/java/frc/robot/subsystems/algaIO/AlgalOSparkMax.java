package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.*;

public class AlgalOSparkMax implements AlgaIO {
    private final SparkMax motor;

    public AlgalOSparkMax(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}