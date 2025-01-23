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
        VELOCITY_SETPOINT,
        POSITION_SETPOINT,
    };
    CONTROL_MODE controlMode = CONTROL_MODE.DISABLED;

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    // odometry
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d fieldPosition = new Pose2d();
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    
    // position control
    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    private PIDController xController = new PIDController(2.7, 0.05, 0.12);
    private PIDController yController = new PIDController(2.7, 0.05, 0.12);
    private PIDController thetaaController = new PIDController(3, 0, 0.3);
    
    // velocity control
    private ChassisSpeeds chassisSetpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public Drive(
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO, 
        GyroIO gyroIO
    ) {
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        for (Module module : modules) {
            module.setBrakeMode();
        }

        xController.setTolerance(.035);
        yController.setTolerance(.035);
        thetaaController.setTolerance(.025);
        thetaaController.enableContinuousInput(-Math.PI, Math.PI);

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
            gyroInputs.connected ? new Rotation2d(gyroInputs.yawPosition.in(Radians)) : fieldPosition.getRotation(),
            getModulePositions()
        );
        Logger.recordOutput("Odometry/FieldPosition", getPose());   
        
        // update module inputs
        for (Module module : modules) {
            module.periodic();
        }
        
        // run modules
        switch (controlMode) {
            case DISABLED:
                for (Module module : modules) {
                    module.stop();
                }
                Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
                Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
                return;
            case POSITION_SETPOINT:
                // get and add PID outputs
                double xPID = xController.atSetpoint() ? 0 : xController.calculate(getPose().getX(), positionSetpoint.getX());
                double yPID = yController.atSetpoint() ? 0 : yController.calculate(getPose().getY(), positionSetpoint.getY());
                double thetaaPID = thetaaController.atSetpoint() ? 0 : thetaaController.calculate(getPose().getRotation().getRadians(),
                    positionSetpoint.getRotation().getRadians()
                );
                chassisSetpoint = new ChassisSpeeds(
                    twistSetpoint.dx + xPID,
                    twistSetpoint.dy + yPID,
                    twistSetpoint.dtheta + thetaaPID
                );

                // FOC
                chassisSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSetpoint.vxMetersPerSecond,
                    chassisSetpoint.vyMetersPerSecond,
                    chassisSetpoint.omegaRadiansPerSecond,
                    getYaw() // not with field relative, because the setpoint is already generated with it in mind
                );

                // fallthrough to CHASSIS_SETPOINT case, no break statement needed
            case VELOCITY_SETPOINT: // chassis is just the drivebase
                // Brief explanation here:
                // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html
                // For more detail, see chapter 10 here:
                // https://file.tavsys.net/control/controls-engineering-in-frc.pdf
                // Purpose: accounts for continuous movement along an arc instead of a discrete
                // straight line, avoids skew
                // More detail here:
                // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/47
                Twist2d setpointTwist = new Pose2d().log( // ! why use a new Pose2d every time
                    new Pose2d(
                        chassisSetpoint.vxMetersPerSecond * Constants.PERIOD,
                        chassisSetpoint.vyMetersPerSecond * Constants.PERIOD,
                        new Rotation2d(chassisSetpoint.omegaRadiansPerSecond * Constants.PERIOD)
                    )
                );

                // Takes the twist deltas and converts them to velocities
                ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
                    setpointTwist.dx / Constants.PERIOD,
                    setpointTwist.dy / Constants.PERIOD,
                    setpointTwist.dtheta / Constants.PERIOD
                );

                // Uses the IK to convert from chassis velocities to individual swerve module positions/velocities
                Logger.recordOutput("Target Velocity", adjustedSpeeds);
                SwerveModuleState[] setpointStates = HardwareConstants.KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

                // Ensure a module isnt trying to go faster than max // ! is max_linear_speed the max speed of a module or of the wheel???
                SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, HardwareConstants.MAX_LINEAR_SPEED);

                // Set to last angles if zero
                if (adjustedSpeeds.vxMetersPerSecond == 0
                    && adjustedSpeeds.vyMetersPerSecond == 0
                    && adjustedSpeeds.omegaRadiansPerSecond == 0)
                 {
                    for (int i = 0; i < 4; i++) {
                        setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
                    }
                }

                lastSetpointStates = setpointStates;

                // Send setpoints to modules
                SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
                for (int i = 0; i < 4; i++) {
                    optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
                }

                Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;
        }
    }

    // ————— functions for running modules ————— //

    public void stop() {
        runVelocity(new ChassisSpeeds()); // ! why not set controlmode to disabled
    }

    public void runPosition(SwerveSample sample){
        controlMode = CONTROL_MODE.POSITION_SETPOINT;
        positionSetpoint = sample.getPose();
        twistSetpoint = sample.getChassisSpeeds().toTwist2d(1); // ! pay attention to this (dtSeconds)
    }

    public void runVelocity(ChassisSpeeds speeds) {
        controlMode = CONTROL_MODE.VELOCITY_SETPOINT;
        chassisSetpoint = speeds;
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
                modules[i].getPositionMeters(),
                modules[i].getAngle()
            );
        }
        return wheelPositions;
    }

    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                modules[i].getPositionMeters() - lastModulePositionsMeters[i],
                modules[i].getAngle()
            );
            lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        return wheelDeltas;
    }
}