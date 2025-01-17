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
        inputs.yawPosition = Degrees.of(-navx.getYaw()); // negative cuz its switched for some reason

        inputs.rollVelocity = DegreesPerSecond.of(navx.getRawGyroY());
        inputs.pitchVelocity = DegreesPerSecond.of(navx.getRawGyroX());
        inputs.yawVelocity = DegreesPerSecond.of(navx.getRawGyroZ());
    }
}