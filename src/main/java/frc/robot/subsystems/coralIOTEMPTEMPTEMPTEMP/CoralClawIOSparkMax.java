package frc.robot.subsystems.coralIOTEMPTEMPTEMPTEMP;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class CoralClawIOSparkMax implements CoralClawIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    private final double kP = 10;
    private final double kI = 0;
    private final double kD = 0;

    public CoralClawIOSparkMax(int id){
        motor = new SparkMax(id, MotorType.kBrushed);

        // start config
        motor.setCANTimeout(500);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

        motorConfig.signals.primaryEncoderPositionPeriodMs(20);
        motorConfig.encoder.quadratureAverageDepth(2);
        motorConfig.smartCurrentLimit(30);
        motorConfig.voltageCompensation(12);
        motorConfig.idleMode(IdleMode.kBrake);

        motor.setCANTimeout(0);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(Voltage volts){
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setPosition(Angle position){
        motor.getClosedLoopController().setReference(
            position.in(Rotations),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void updateInputs(CoralClawIOInputs inputs) {
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}