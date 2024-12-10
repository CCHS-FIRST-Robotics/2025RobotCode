package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class DriveSideIOTalonSRX implements DriveSideIO {
    private final TalonSRX motor1, motor2;
    private final PIDController PID;
    private final SimpleMotorFeedforward F;
    int encoderTicks = 4096;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0;
    private final double kV = 0.135; // at 88.83 rotations per second, output 12 volts
    private final double kA = 0;

    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    DriveSideIOInputs inputs = new DriveSideIOInputs();
    
    public DriveSideIOTalonSRX(int id1, int id2, boolean isInverted, boolean isInPhase){
        motor1 = new TalonSRX(id1);
        motor2 = new TalonSRX(id2);
        PID = new PIDController(kP, kI, kD);
        F = new SimpleMotorFeedforward(kS, kV, kA);

        motor1.configFactoryDefault();
        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor1.setSelectedSensorPosition(0);
        motor1.setSensorPhase(isInPhase);
        motor1.setInverted(isInverted);
        motor2.follow(motor1);
        motor2.setInverted(InvertType.FollowMaster);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
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
        inputs.motor1Current = motor1.getStatorCurrent();
        inputs.motor1Voltage = motor1.getMotorOutputVoltage();
        inputs.motor1Temperature = motor1.getTemperature();

        inputs.wheelPosition = motor1.getSelectedSensorPosition() / 4096;
        inputs.wheelVelocity = motor1.getSelectedSensorVelocity() * 10 / 4096;
        inputs.motor1Position = inputs.wheelPosition * Constants.GEAR_RATIO;
        inputs.motor1Velocity = inputs.wheelVelocity * Constants.GEAR_RATIO;

        inputs.currentSetpoint = currentSetpoint.in(RotationsPerSecond);
        inputs.distanceTraveled = inputs.wheelPosition * Constants.WHEEL_CIRCUMFERENCE.in(Meters);

        this.inputs = inputs;
    }
}