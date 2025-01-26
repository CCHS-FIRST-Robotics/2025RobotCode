package frc.robot.subsystems.coralIO;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase{
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    // public Command getSetPositionCommand(ElevatorPosition position){
    //     return new InstantCommand(() -> setPosition(position)).until(() -> atGoal());
    // }
}