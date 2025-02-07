package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class Claw {
    Solenoid solenoid;
    
    public Claw(Solenoid solenoid) {
        this.solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_PIN);
    }

    public void grab() {
        solenoid.set(true);
    }

    public void release() {
        solenoid.set(false);
    }
}
