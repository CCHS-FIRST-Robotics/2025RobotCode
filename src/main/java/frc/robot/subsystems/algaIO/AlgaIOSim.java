package frc.robot.subsystems.algaIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.VirtualConstants;

public class AlgaIOSim implements AlgaIO {
    private final DCMotorSim motor;
    private final double MOI = 1; // MOI of a rod = (M(R^2))/2

    private Voltage appliedVolts = Volts.of(0);
    
    public AlgaIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), MOI, 1),
            DCMotor.getKrakenX60(1)
        );
    }
    
    @Override
    public void setAlgaVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(AlgaIOInputs inputs) {
        motor.update(VirtualConstants.PERIOD);

        inputs.algaVoltage = appliedVolts.in(Volts);
        inputs.algaCurrent = motor.getCurrentDrawAmps();
        inputs.algaTemperature = 0;
    }
}