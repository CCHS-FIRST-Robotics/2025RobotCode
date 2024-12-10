package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class Drive extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final DifferentialDriveOdometry odometry;
    private Pose2d robotPose2d = new Pose2d();
    
    private final DriveSideIO lIO, rIO;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    private final DriveSideIOInputsAutoLogged lInputs = new DriveSideIOInputsAutoLogged();
    private final DriveSideIOInputsAutoLogged rInputs = new DriveSideIOInputsAutoLogged();

    public Drive(
        GyroIO gyroIO, 
        DriveSideIO leftIO, 
        DriveSideIO rightIO
    ) {
        this.lIO = leftIO;
        this.rIO = rightIO;
        this.gyroIO = gyroIO;
        this.odometry = new DifferentialDriveOdometry(
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d(0, 0, new Rotation2d())
        );
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        robotPose2d = odometry.update(
            gyroInputs.connected ? gyroInputs.rotation2D: new Rotation2d(),
            lInputs.distanceTraveled, 
            rInputs.distanceTraveled
        );
        lIO.updateInputs(lInputs);
        rIO.updateInputs(rInputs);

        Logger.processInputs("Gyro ", gyroInputs);
        Logger.recordOutput("RobotPose2D", robotPose2d);
        Logger.processInputs("Left side ", lInputs);
        Logger.processInputs("Right side ", rInputs);
    }

    public void setVoltage(Measure<Voltage> lVolts, Measure<Voltage> rVolts){
        lIO.setVoltage(lVolts);
        rIO.setVoltage(rVolts);
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        Measure<Velocity<Angle>> leftMotorVelocity = RotationsPerSecond.of(
            wheelSpeeds.leftMetersPerSecond 
            / Constants.WHEEL_CIRCUMFERENCE.in(Meters) // to get rotations per second of the wheel
            * Constants.GEAR_RATIO // to get rotations per second of the motor
        );
        Measure<Velocity<Angle>> rightMotorVelocity = RotationsPerSecond.of(
            wheelSpeeds.rightMetersPerSecond 
            / Constants.WHEEL_CIRCUMFERENCE.in(Meters) // to get rotations per second of the wheel
            * Constants.GEAR_RATIO // to get rotations per second of the motor
        );

        lIO.setVelocity(leftMotorVelocity); // should be 88.83 rotations per second
        rIO.setVelocity(rightMotorVelocity);
    }
}