package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import choreo.trajectory.SwerveSample;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.*;

public class Drive extends SubsystemBase {
    public enum CONTROL_MODE {
        DISABLED,
        POSITION,
        VELOCITY
    };
    CONTROL_MODE controlMode = CONTROL_MODE.DISABLED;

    private final Module[] modules = new Module[4];

    // odometry
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d fieldPosition = new Pose2d();
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    
    // position control
    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    private final PIDController xController = new PIDController(2.7, 0.05, 0.12);
    private final PIDController yController = new PIDController(2.7, 0.05, 0.12);
    private final PIDController headingController = new PIDController(3, 0, 0.3);
    
    // velocity control
    private ChassisSpeeds speeds = new ChassisSpeeds();

    public Drive(
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO, 
        GyroIO gyroIO
    ) {
        modules[0] = new Module(flModuleIO, 1);
        modules[1] = new Module(frModuleIO, 2);
        modules[2] = new Module(blModuleIO, 3);
        modules[3] = new Module(brModuleIO, 4);

        xController.setTolerance(.035);
        yController.setTolerance(.035);
        headingController.setTolerance(.025);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.gyroIO = gyroIO;
        poseEstimator = new SwerveDrivePoseEstimator(
            HardwareConstants.KINEMATICS, 
            new Rotation2d(), 
            getModulePositions(), 
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = CONTROL_MODE.DISABLED;
        }

        // update odometry
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("gyro", gyroInputs);
        fieldPosition = fieldPosition.exp(HardwareConstants.KINEMATICS.toTwist2d(getModuleDeltas()));
        poseEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : fieldPosition.getRotation(),
            getModulePositions()
        );
        Logger.recordOutput("robotPose", getPose());   
        
        // update module inputs
        for (Module module : modules) {
            module.periodic();
        }
        
        // run modules
        switch (controlMode) {
            case DISABLED:
                // set all modules' voltage to 0
                for (Module module : modules) {
                    module.stop();
                }
                break;
            case POSITION:
                // get PIDs
                double xPID = xController.calculate(getPose().getX(), positionSetpoint.getX()); // ! (xController.atSetpoint() ? 0 :) was removed
                double yPID = yController.calculate(getPose().getY(), positionSetpoint.getY());
                double headingPID = headingController.calculate(getPose().getRotation().getRotations(), positionSetpoint.getRotation().getRotations());
                
                // create chassisspeeds object with FOC
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    twistSetpoint.dx + xPID,
                    twistSetpoint.dy + yPID,
                    twistSetpoint.dtheta + headingPID,
                    getYaw() // not getYawWithAllianceRotation(), because the setpoint is already generated with it in mind
                );
            case VELOCITY: // fallthrough to VELOCITY case, no break statement needed above
                ChassisSpeeds.discretize(speeds, Constants.PERIOD); // more detail: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
                SwerveModuleState[] moduleStates = HardwareConstants.KINEMATICS.toSwerveModuleStates(speeds); // convert speeds to module states
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, HardwareConstants.MAX_LINEAR_SPEED); // renormalize wheel speeds
                
                // run modules
                for (int i = 0; i < 4; i++) {
                    modules[i].runState(moduleStates[i]);
                }
                break;
        }
    }

    // ————— functions for running modules ————— //

    public void stop() {
        // runVelocity(new ChassisSpeeds()); // ! test with replacing this with just controlMode = CONTROL_MODE.DISABLED
        controlMode = CONTROL_MODE.DISABLED; // ! doesn't seem to make much difference in sim
    }

    public void runPosition(SwerveSample sample){
        controlMode = CONTROL_MODE.POSITION;
        positionSetpoint = sample.getPose();
        twistSetpoint = sample.getChassisSpeeds().toTwist2d(1);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        controlMode = CONTROL_MODE.VELOCITY;
        this.speeds = speeds;
    }

    // ————— functions for odometry ————— //

    public void resetPoseEstimator(Pose2d pose){
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(Math.PI) : 
            new Rotation2d(0)
        );
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelPositions[i] = new SwerveModulePosition(
                modules[i].getDistanceTraveled(),
                modules[i].getWrappedAngle()
            );
        }
        return wheelPositions;
    }

    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                modules[i].getDistanceTraveled() - lastModulePositionsMeters[i],
                modules[i].getWrappedAngle()
            );
            lastModulePositionsMeters[i] = modules[i].getDistanceTraveled();
        }
        return wheelDeltas;
    }
}