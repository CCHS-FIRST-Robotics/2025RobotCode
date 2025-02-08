package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Claw {
    SparkMax clawDriver;
    
    public Claw(SparkMax clawDriver) {
        this.clawDriver = new SparkMax(Constants.CLAW_ID, MotorType.kBrushless);
    }

    public void grab() {
        clawDriver.setVoltage(2);
    }

    public void release() {
        clawDriver.setVoltage(-2);
    }

    public void stop() {
        clawDriver.setVoltage(0);
    }
}
