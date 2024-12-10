package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class DriveSideIOSim implements DriveSideIO {
    private final DCMotorSim motor;
    private final PIDController PID;
    private final SimpleMotorFeedforward F;

    // recalc gains are in volts per (meters per second)
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0;
    private final double kV = 3.6 * Constants.WHEEL_CIRCUMFERENCE.in(Meters) / Constants.GEAR_RATIO;
    private final double kA = 0.26 * Constants.WHEEL_CIRCUMFERENCE.in(Meters) / Constants.GEAR_RATIO;

    Measure<Voltage> appliedVolts = Volts.of(0);
    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    DriveSideIOInputs inputs = new DriveSideIOInputs();
    
    public DriveSideIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kV, kA),
            DCMotor.getCIM(2), // or just divide kA by 2, since kT * torque = volts and kT = kA
            Constants.GEAR_RATIO
        );
        PID = new PIDController(kP, kI, kD);
        F = new SimpleMotorFeedforward(kS, kV, kA);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        this.setVoltage(Volts.of(
            PID.calculate(inputs.motor1Velocity, velocity.in(RotationsPerSecond))
            + F.calculate(velocity.in(RotationsPerSecond))
        ));
        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motor1Current = motor.getCurrentDrawAmps();
        inputs.motor1Voltage = appliedVolts.in(Volts);
        inputs.motor1Temperature = 0;

        inputs.motor1Position = motor.getOutput(0); // gets the position in the units of the gains you give it
        inputs.motor1Velocity = motor.getOutput(1);
        inputs.wheelPosition = inputs.motor1Position / Constants.GEAR_RATIO;
        inputs.wheelVelocity = inputs.motor1Velocity / Constants.GEAR_RATIO;

        inputs.currentSetpoint = currentSetpoint.in(RotationsPerSecond);
        inputs.distanceTraveled = inputs.wheelPosition * Constants.WHEEL_CIRCUMFERENCE.in(Meters);

        this.inputs = inputs;
    }
}