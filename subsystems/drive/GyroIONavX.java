package frc.robot.subsystems.drive  ;

import com.kauailabs.navx.frc.*;

public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        navx = new AHRS();
        navx.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.rotation2D = navx.getRotation2d();
    }
}