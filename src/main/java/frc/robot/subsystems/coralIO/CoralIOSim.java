package frc.robot.subsystems.coralIO;

import edu.wpi.first.units.measure.*;

// ! this file does not exist
public class CoralIOSim implements CoralIO{
    public CoralIOSim() {}

    @Override
    public void setElevatorVoltage(Voltage volts) {}

    @Override
    public void setElevatorPosition(Angle position) {}

    @Override
    public boolean elevatorAtSetpoint(Angle position) {
        return true;
    }

    @Override
    public void setArmVoltage(Voltage volts) {}

    @Override
    public void setArmPosition(Angle position) {}

    public boolean armAtSetpoint(Angle position) {
        return true;
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {}
    }