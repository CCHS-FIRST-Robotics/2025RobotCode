package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.Constants;

public class AlgaIOSim implements AlgaIO {
    private final DCMotorSim motor;
    
    private final double MOI = 0; // MOI of a rod = (M(R^2))/2

    Voltage appliedVolts = Volts.of(0);
    Angle currentSetpoint = Rotations.of(0);
    AlgaIOInputs inputs = new AlgaIOInputs();
    
    public AlgaIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), MOI, 1),
            DCMotor.getKrakenX60(1)
        );
    }
    
    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motorVoltage = appliedVolts.in(Volts);
        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorVelocity = motor.getOutput(1); // gets the velocity in the units of the gains you give it
        inputs.motorTemperature = 0;

        this.inputs = inputs;
    }
}