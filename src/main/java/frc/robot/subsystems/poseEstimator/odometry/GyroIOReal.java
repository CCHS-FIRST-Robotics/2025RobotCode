package frc.robot.subsystems.poseEstimator.odometry;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class GyroIOReal implements GyroIO {
    private final AHRS navx;

    public GyroIOReal() {
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        
        inputs.roll = Degrees.of(navx.getRoll()).in(Rotations);
        inputs.pitch = Degrees.of(navx.getPitch()).in(Rotations);
        inputs.yaw = Degrees.of(-navx.getYaw()).in(Rotations); // navx is flipped
        inputs.rollVelocity = DegreesPerSecond.of(navx.getRawGyroY()).in(RotationsPerSecond);
        inputs.pitchVelocity = DegreesPerSecond.of(navx.getRawGyroX()).in(RotationsPerSecond);
        inputs.yawVelocity = DegreesPerSecond.of(navx.getRawGyroZ()).in(RotationsPerSecond);
    }
}