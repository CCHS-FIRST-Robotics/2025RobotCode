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
        
        inputs.roll = Degrees.of(navx.getRoll()).in(Rotations);
        inputs.pitch = Degrees.of(navx.getPitch()).in(Rotations);
        inputs.yaw = Degrees.of(-navx.getYaw()).in(Rotations); // navx is flipped
    }
}