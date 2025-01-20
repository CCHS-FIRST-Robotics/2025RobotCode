package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareConstants;
import frc.robot.utils.PoseEstimator;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private ChassisSpeeds chassisSetpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    // position control
    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    private ArrayList<Pose2d> positionTrajectory = new ArrayList<Pose2d>();
    private ArrayList<Twist2d> twistTrajectory = new ArrayList<Twist2d>();
    private int trajectoryCounter = -1;

    private double kPx = 2.7;
    private double kIx = 0.05;
    private double kDx = 0.12;
    private double kPy = 2.7;
    private double kIy = 0.05;
    private double kDy = 0.12;
    private double kPthetaaaaa = 3;
    private double kIthetaaaaa = 0;
    private double kDthetaaaaa = .3;
    private PIDController xController = new PIDController(kPx, kIx, kDx);
    private PIDController yController = new PIDController(kPy, kIy, kDy);
    private PIDController thetaaaaaController = new PIDController(kPthetaaaaa, kIthetaaaaa, kDthetaaaaa);

    /*
     * ODOMETRY
     */
    private PoseEstimator poseEstimator;
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Pose2d fieldPosition = new Pose2d(); // used if gyro isn't connected

    public enum CONTROL_MODE {
        DISABLED,
        CHASSIS_SETPOINT,
        POSITION_SETPOINT,
    };
    CONTROL_MODE controlMode = CONTROL_MODE.DISABLED;

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        
        for (Module module : modules) {
            module.setBrakeMode();
        }
        
        xController.setTolerance(.035);
        yController.setTolerance(.035);
        thetaaaaaController.setTolerance(.025);
        thetaaaaaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = CONTROL_MODE.DISABLED;
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        
        for (Module module : modules) {
            module.periodic();
        }

        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }

        /*
         * UPDATE ODOMETRY
         */
        Twist2d twist = HardwareConstants.KINEMATICS.toTwist2d(getModuleDeltas());
        Rotation2d gyroYaw = new Rotation2d(gyroInputs.yawPosition.in(Radians));
        if (gyroInputs.connected) {
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        }
        lastGyroYaw = gyroYaw;
        fieldPosition = fieldPosition.exp(twist);
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                (gyroInputs.connected ? gyroYaw : fieldPosition.getRotation()),
                getModulePositions());
        Logger.recordOutput("Odometry/FieldPosition", getPose());        

        switch (controlMode) {
            case DISABLED:
                for (var module : modules) {
                    module.stop();
                }
                Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
                Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
                return;
            case POSITION_SETPOINT:
                if (trajectoryCounter == -1){ // if no available trajectory
                    break;
                }
                if (trajectoryCounter > positionTrajectory.size() - 1) { // if at the end of trajectory, hold the last setpoint
                    trajectoryCounter = positionTrajectory.size() - 1;
                }

                positionSetpoint = positionTrajectory.get(trajectoryCounter);
                twistSetpoint = twistTrajectory.get(trajectoryCounter);

                // get and add PID outputs
                double xPID = xController.atSetpoint() ? 0 : xController.calculate(getPose().getX(), positionSetpoint.getX());
                double yPID = yController.atSetpoint() ? 0 : yController.calculate(getPose().getY(), positionSetpoint.getY());
                double thetaaaaaPID = thetaaaaaController.atSetpoint() ? 0 : thetaaaaaController.calculate(getPose().getRotation().getRadians(),
                    positionSetpoint.getRotation().getRadians()
                );
                chassisSetpoint = new ChassisSpeeds(
                    twistSetpoint.dx + xPID,
                    twistSetpoint.dy + yPID,
                    twistSetpoint.dtheta + thetaaaaaPID
                );

                // FOC
                chassisSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSetpoint.vxMetersPerSecond,
                    chassisSetpoint.vyMetersPerSecond,
                    chassisSetpoint.omegaRadiansPerSecond,
                    getYaw() // not with field relative, because the setpoint is already generated with it in mind
                );

                trajectoryCounter++;
                // fallthrough to CHASSIS_SETPOINT case, no break statement needed
            case CHASSIS_SETPOINT: // chassis is just the drivebase
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

    public void followTrajectory(SwerveSample sample){
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaaaaaController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
    }

    public void resetOdometry(Pose2d pose){
        
    }

    public void runVelocity(ChassisSpeeds speeds) {
        // Since DriveWithJoysticks is the default command and MoveToPose runs once
        // Keep drive running the position trajectory unless overridden (driver sets a
        // nonzero speed with joysticks)
        if (controlMode == CONTROL_MODE.POSITION_SETPOINT && speedsEqual(speeds, new ChassisSpeeds())) {
            return;
        }
        controlMode = CONTROL_MODE.CHASSIS_SETPOINT;
        chassisSetpoint = speeds;
    }

    public static boolean speedsEqual(ChassisSpeeds speeds, ChassisSpeeds other) {
        return (speeds.vxMetersPerSecond == other.vxMetersPerSecond &&
                speeds.vyMetersPerSecond == other.vyMetersPerSecond &&
                speeds.omegaRadiansPerSecond == other.omegaRadiansPerSecond);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
        // ! why not set controlmode to disabled
    }

    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPosition.in(Radians));
    }

    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPosition.in(Radians));
    }

    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        // make field relative to red if on red team
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(Math.PI) : 
            new Rotation2d(0)
        ); 
    }

    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
                    modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        return wheelDeltas;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] wheelPos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelPos[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters()),
                    modules[i].getAngle());
        }
        return wheelPos;
    }

    // ! are these seriously needed
    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public PIDController getthetaaaaaController() {
        return thetaaaaaController;
    }

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public Pose2d getPose() {
        return poseEstimator.getPoseEstimate();
    }
}