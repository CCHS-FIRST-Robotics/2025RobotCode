package frc.robot.subsystems.coralIOTEMPTEMPTEMPTEMP;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.coralIO.CoralArmIOInputsAutoLogged;
import frc.robot.subsystems.coralIO.CoralClawIOInputsAutoLogged;
import frc.robot.subsystems.coralIO.CoralWristIOInputsAutoLogged;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
    private final CoralClawIO claw;
    private final CoralWristIO wrist;
    private final CoralArmIO arm;

    private final CoralClawIOInputsAutoLogged clawInputs = new CoralClawIOInputsAutoLogged();
    private final CoralWristIOInputsAutoLogged wristInputs = new CoralWristIOInputsAutoLogged();
    private final CoralArmIOInputsAutoLogged armInputs = new CoralArmIOInputsAutoLogged();

    public Coral(CoralClawIO claw, CoralWristIO wrist, CoralArmIO arm) {
        this.claw = claw;
        this.wrist = wrist;
        this.arm = arm;
    }

    @Override
    public void periodic() {
        claw.updateInputs(clawInputs);
        Logger.processInputs("coralIO", clawInputs);
        wrist.updateInputs(wristInputs);
        Logger.processInputs("coralIO", wristInputs);
        arm.updateInputs(armInputs);
        Logger.processInputs("coralIO", armInputs);
    }

    // set 
}