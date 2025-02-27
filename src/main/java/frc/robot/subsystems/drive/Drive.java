package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import frc.robot.constants.*;
import frc.robot.subsystems.PoseEstimator.*;

public class Drive extends SubsystemBase {
    public enum DRIVE_MODE {
        DISABLED,
        POSITION,
        VELOCITY,
        CHARACTERIZING
    };
    private DRIVE_MODE controlMode = DRIVE_MODE.DISABLED;

    private final Module[] modules = new Module[4];

    // odometry
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private PoseEstimator poseEstimator;
    private Pose2d fieldPosition = new Pose2d();
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

    // charactarization
    private Voltage characterizationVolts = Volts.of(0);
    private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1), // ramp rate
            Volts.of(2), // step voltage
            Seconds.of(3), // timeout
            (state) -> Logger.recordOutput("drive/sysIdState", state.toString()) // send the data to advantagekit
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> this.runCharacterization(volts), // characterization supplier
            null, // no log consumer since advantagekit records the data
            this
        )
    );

    // position
    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    private final PIDController xController = new PIDController(3, 0, 0);
    private final PIDController yController = new PIDController(3, 0, 0);
    private final PIDController oController = new PIDController(3, 0, 0);
    // private final PIDController xController = new PIDController(50, 0, 0);
    // private final PIDController yController = new PIDController(50, 0, 0);
    // private final PIDController oController = new PIDController(30, 0, 0);
    
    // velocity
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

        oController.enableContinuousInput(-Math.PI, Math.PI); // ! is this even necessary

        this.gyroIO = gyroIO;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = DRIVE_MODE.DISABLED;
        }

        // ————— odometry ————— //

        // gyro
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("gyro", gyroInputs);
        // module states
        SwerveModuleState[] moduleStatesOutput = getModuleStates();
        Logger.recordOutput("outputs/drive/moduleStatesOutput", moduleStatesOutput);
        // chassisspeeds
        ChassisSpeeds speedsOutput = PhysicalConstants.KINEMATICS.toChassisSpeeds(moduleStatesOutput);
        Logger.recordOutput("outputs/drive/speedsOutput", speedsOutput);
        // pose
        fieldPosition = fieldPosition.exp(PhysicalConstants.KINEMATICS.toTwist2d(getModuleDeltas()));
        poseEstimator.updateWithOdometry(
            Timer.getFPGATimestamp(),
            gyroInputs.connected ? getYaw() : fieldPosition.getRotation(),
            getModulePositions()
        );
        
        // ————— driving ————— //

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
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case CHARACTERIZING:
                for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(characterizationVolts);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case POSITION:
                System.out.println("1, " + positionSetpoint.getX());
                System.out.println("2, " + positionSetpoint.getY());
                System.out.println("3, " + positionSetpoint.getRotation().getRadians());
                // get PIDs
                double xPID = xController.calculate(poseEstimator.getPose().getX(), positionSetpoint.getX());
                double yPID = yController.calculate(poseEstimator.getPose().getY(), positionSetpoint.getY());
                double oPID = oController.calculate(poseEstimator.getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());
                
                System.out.println("4, " + xPID);
                System.out.println("5, " + yPID);
                System.out.println("6, " + oPID);

                // create chassisspeeds object with FOC
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    twistSetpoint.dx + xPID,
                    twistSetpoint.dy + yPID,
                    twistSetpoint.dtheta + oPID,
                    getYaw() // not getYawWithAllianceRotation(), because the setpoint is already generated with it in mind
                );
                // fallthrough to VELOCITY case; no break statement needed
            case VELOCITY: 
                speeds = ChassisSpeeds.discretize(speeds, VirtualConstants.PERIOD); // explaination: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30                
                SwerveModuleState[] moduleStates = PhysicalConstants.KINEMATICS.toSwerveModuleStates(speeds); // convert speeds to module states
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, PhysicalConstants.MAX_ALLOWED_LINEAR_SPEED); // renormalize wheel speeds

                // run modules
                for (int i = 0; i < 4; i++) {
                    modules[i].runState(moduleStates[i]);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", moduleStates);
                break;
        }
    }

    // ————— functions for running modules ————— //

    public void stop() {
        controlMode = DRIVE_MODE.DISABLED;
    }

    public void runCharacterization(Voltage volts){
        controlMode = DRIVE_MODE.CHARACTERIZING;
        characterizationVolts = volts;
    }

    public void runPosition(Pose2d pose){
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = pose;
        twistSetpoint = new Twist2d();
    }

    public void runAutoPosition(SwerveSample sample){
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = sample.getPose();
        twistSetpoint = sample.getChassisSpeeds().toTwist2d(0.02);
    }

    public void runVelocity(ChassisSpeeds speedsInput) {
        controlMode = DRIVE_MODE.VELOCITY;
        speeds = speedsInput;
    }

    // ————— functions for sysid ————— // 

    public Command sysIdFull(){
        return driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    // ————— functions for odometry ————— //

    public void setPoseEstimator(PoseEstimator poseEstimator){
        this.poseEstimator = poseEstimator;
    }

    public void resetPoseEstimator(Pose2d pose){
        poseEstimator.resetPosition(pose, getModulePositions());
    }

    public Rotation2d getYaw() {
        return gyroInputs.connected ? new Rotation2d(Rotations.of(gyroInputs.yaw).in(Radians)) : poseEstimator.getPose().getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(0) : 
            new Rotation2d(Math.PI)
        );
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelPositions[i] = new SwerveModulePosition(
                modules[i].getDistance(),
                modules[i].getAngle()
            );
        }
        return wheelPositions;
    }

    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                modules[i].getDistance() - lastModulePositionsMeters[i],
                modules[i].getAngle()
            );
            lastModulePositionsMeters[i] = modules[i].getDistance();
        }
        return wheelDeltas;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = modules[i].getState();
        }
        return moduleStates;
    }
}