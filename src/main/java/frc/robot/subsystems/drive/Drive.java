package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import choreo.trajectory.SwerveSample;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;

public class Drive extends SubsystemBase {
    private enum DRIVE_MODE {
        DISABLED,
        POSITION,
        VELOCITY,
        CHARACTERIZING
    };
    private DRIVE_MODE controlMode = DRIVE_MODE.DISABLED;

    private final Module[] modules = new Module[4];

    // ————— odometry ————— //
    private PoseEstimator poseEstimator;
    private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

    // ————— charactarization ————— //

    private Voltage[] characterizationVolts = {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)};
    private Angle[] characterizationPositions = {Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)};
    private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1), // ramp rate
            Volts.of(2), // step voltage
            Seconds.of(5), // timeout
            (state) -> Logger.recordOutput("drive/sysIdState", state.toString()) // send the data to advantagekit
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> this.runCharacterization(
                new Voltage[] {volts, volts, volts, volts}, 
                new Angle[] {Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)}
            ), // characterization supplier
            null, // no log consumer needed since advantagekit records the data
            this
        )
    );

    // ————— position ————— // 

    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    // manual
    private final PIDController xPID = new PIDController(3, 0, 0);
    private final PIDController yPID = new PIDController(3, 0, 0);
    private final PIDController oPID = new PIDController(3, 0, 0);
    // choreo
    // private final PIDController xController = new PIDController(50, 0, 0);
    // private final PIDController yController = new PIDController(50, 0, 0);
    // private final PIDController oController = new PIDController(30, 0, 0);
    
    // ————— velocity ————— //
    private ChassisSpeeds speeds = new ChassisSpeeds();

    public Drive(
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        modules[0] = new Module(flModuleIO, 1);
        modules[1] = new Module(frModuleIO, 2);
        modules[2] = new Module(blModuleIO, 3);
        modules[3] = new Module(brModuleIO, 4);

        oPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = DRIVE_MODE.DISABLED;
        }

        // ————— odometry ————— //

        updateModuleDeltas();
        // module states
        SwerveModuleState[] moduleStatesOutput = getModuleStates();
        Logger.recordOutput("outputs/drive/moduleStatesOutput", moduleStatesOutput);
        // chassisspeeds
        ChassisSpeeds speedsOutput = PhysicalConstants.KINEMATICS.toChassisSpeeds(moduleStatesOutput);
        Logger.recordOutput("outputs/drive/speedsOutput", speedsOutput);

        // ————— driving ————— //

        // update module inputs
        for (Module module : modules) {
            module.periodic();
        }

        // run modules 
        switch (controlMode) {
            case DISABLED:
                // set all module voltages to 0
                for (Module module : modules) {
                    module.stop();
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case CHARACTERIZING:
                for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(characterizationVolts[i], characterizationPositions[i]);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case POSITION:
                // get PIDs
                double xOutput = xPID.calculate(poseEstimator.getPose().getX(), positionSetpoint.getX());
                double yOutput = yPID.calculate(poseEstimator.getPose().getY(), positionSetpoint.getY());
                double oOutput = oPID.calculate(poseEstimator.getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());

                // create chassisspeeds object with FOC
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    twistSetpoint.dx + xOutput,
                    twistSetpoint.dy + yOutput,
                    twistSetpoint.dtheta + oOutput,
                    poseEstimator.getRawYaw() // not getYawWithAllianceRotation(), because the setpoint is already generated with it in mind
                );
                // fallthrough to VELOCITY case; no break statement needed
            case VELOCITY: 
                speeds = ChassisSpeeds.discretize(speeds, VirtualConstants.PERIOD); // explaination: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30                
                
                // let the wheels go back to being straight when there is no input, instead of holding their last angle
                if (speeds.vxMetersPerSecond == 0
                 && speeds.vxMetersPerSecond == 0
                 && speeds.vxMetersPerSecond == 0
                ) {
                    PhysicalConstants.KINEMATICS.resetHeadings(new Rotation2d[] {new Rotation2d(0), new Rotation2d(0), new Rotation2d(0), new Rotation2d(0)});
                }
                
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

    public void runCharacterization(Voltage[] volts, Angle[] positions) {
        controlMode = DRIVE_MODE.CHARACTERIZING;
        characterizationVolts = volts;
        characterizationPositions = positions;
    }

    public void runPosition(Pose2d pose) {
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = pose;
        twistSetpoint = new Twist2d();
    }

    public void runAutoPosition(SwerveSample sample) {
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = sample.getPose();
        twistSetpoint = sample.getChassisSpeeds().toTwist2d(0.02);
    }

    public void runVelocity(ChassisSpeeds speedsInput) {
        controlMode = DRIVE_MODE.VELOCITY;
        speeds = speedsInput;
    }

    // ————— functions for sysid ————— // 

    public Command sysIdFull() {
        return driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    // ————— functions for odometry ————— //

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                modules[i].getDistance(),
                modules[i].getAngle()
            );
        }
        return modulePositions;
    }

    public void updateModuleDeltas() {
        for (int i = 0; i < 4; i++) {
            moduleDeltas[i] = new SwerveModulePosition(
                modules[i].getDistance() - lastModulePositionsMeters[i],
                modules[i].getAngle()
            );
            lastModulePositionsMeters[i] = modules[i].getDistance();
        }
    }

    public SwerveModulePosition[] getModuleDeltas() {
        return moduleDeltas;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = modules[i].getState();
        }
        return moduleStates;
    }
}