package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.rollPosition = Degrees.of(navx.getRoll());
        inputs.pitchPosition = Degrees.of(navx.getPitch());
        inputs.yawPosition = Degrees.of(-navx.getYaw()); // navx is flipped
    }
}