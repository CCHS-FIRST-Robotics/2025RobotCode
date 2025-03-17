package frc.robot.subsystems.coralIO;

import edu.wpi.first.units.measure.*;

public class CoralIOSim implements CoralIO{
    public CoralIOSim() {}

    @Override
    public void setElevatorVoltage(Voltage volts) {}

    @Override
    public void setElevatorPosition(Angle position) {}

    @Override
    public void setArmVoltage(Voltage volts) {}

    @Override
    public void setArmPosition(Angle position) {}

    @Override
    public void updateInputs(CoralIOInputs inputs) {}
}

// ! old elevator sim code

// package frc.robot.subsystems.coralIO;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.math.controller.*;
// import edu.wpi.first.math.system.plant.*;
// import edu.wpi.first.units.measure.*;
// import frc.robot.constants.Constants;

// public class ArmIOSim implements ArmIO {
//     private final DCMotorSim motor;
//     private final PIDController PID;
//     private final SimpleMotorFeedforward F;

//     // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A2%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A50%2C%22u%22%3A%22in%22%7D
//     private final double kP = 20;
//     private final double kI = 0;
//     private final double kD = 0;
//     private final double kS = 0;
//     private final double kV = 3.01; // V/(m/s) 
//     private final double kA = 0.1; // V/(m/s^2)

//     Voltage appliedVolts = Volts.of(0);
//     Angle currentSetpoint = Rotations.of(0);
//     ElevatorIOInputs inputs = new ElevatorIOInputs();
    
//     public ElevatorIOSim(){
//         motor = new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(kV, kA),
//             DCMotor.getKrakenX60(1)
//         );
//         PID = new PIDController(kP, kI, kD);
//         F = new SimpleMotorFeedforward(kS, kV, kA);
//     }
    
//     @Override
//     public void setVoltage(Voltage volts) {
//         motor.setInputVoltage(volts.in(Volts));
//         appliedVolts = volts;
//     }

//     @Override
//     public void setPosition(Angle position){
//         this.setVoltage(Volts.of(
//             PID.calculate(inputs.motorPosition, position.in(Rotations))
//             + F.calculate(position.in(Rotations))
//         ));
//         currentSetpoint = position;
//     }

//     @Override
//     public void updateInputs(ElevatorIOInputs inputs) {
//         motor.update(Constants.PERIOD);

//         inputs.motorVoltage = appliedVolts.in(Volts);
//         inputs.motorCurrent = motor.getCurrentDrawAmps();
//         inputs.motorPosition = motor.getOutput(0); // gets the position in the units of the gains you give it
//         inputs.motorVelocity = motor.getOutput(1);
//         inputs.motorTemperature = 0;

//         this.inputs = inputs;
//     }
// }