// package frc.robot.subsystems.coralIO;

// import static edu.wpi.first.units.Units.*;
    
// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.units.measure.*;
// public class ArmIOReal {
//     private final TalonFX armMotor;
//     private final SparkMax wristMotor;
//     private final SparkMax clawMotor;

//     private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
//     private final Slot0Configs armPIDF = armConfig.Slot0;
//     private final MotionMagicConfigs armMotionMagicConfig = armConfig.MotionMagic;
//     private final MotionMagicVoltage armMotionMagicVoltage = new MotionMagicVoltage(0);

//     private final SparkMaxConfig wristConfig = new SparkMaxConfig();
//     private final RelativeEncoder encoder;

//     private double kPArm = 0;
//     private double kIArm = 0;
//     private double kDArm = 0;
//     private double kSArm = 0;
//     private double kVArm = 0;
//     private double kAArm = 0;

//     private double kPWrist = 0;
//     private double kIWrist = 0;
//     private double kDWrist = 0;
//     private double kSWrist = 0;
//     private double kVWrist = 0;
//     private double kAWrist = 0;

//     private double kPClaw = 0;
//     private double kIClaw = 0;
//     private double kDClaw = 0;
//     private double kSClaw = 0;
//     private double kVClaw = 0;
//     private double kAClaw = 0;

//     private StatusSignal<Voltage> voltageSignal;
//     private StatusSignal<Current> currentSignal;
//     private StatusSignal<Angle> positionSignal;
//     private StatusSignal<AngularVelocity> velocitySignal;
//     private StatusSignal<Temperature> temperatureSignal;
    
//     public ArmIOReal(int armId, int wristId, int clawId) {
//         armMotor = new TalonFX(armId);

//         armPIDF.kP = kPArm;
//         armPIDF.kI = kIArm;
//         armPIDF.kD = kDArm;
//         armPIDF.kS = kSArm;
//         armPIDF.kV = kVArm;
//         armPIDF.kA = kAArm;

//         armMotionMagicConfig.MotionMagicCruiseVelocity = 100; // motor max rps // ! idk if this is correct
//         armMotionMagicConfig.MotionMagicAcceleration = 1;
//         armMotionMagicConfig.MotionMagicJerk = 1;

//         armMotor.getConfigurator().apply(armConfig);

//         voltageSignal = armMotor.getMotorVoltage();
//         currentSignal = armMotor.getStatorCurrent();
//         positionSignal = armMotor.getPosition();
//         velocitySignal = armMotor.getVelocity();
//         temperatureSignal = armMotor.getDeviceTemp();
//     }

//     @Override
//     public void setPosition(Angle position){
//         motor.setControl(armMotionMagicVoltage.withPosition(position.in(Rotations)).withSlot(0));
//     }

//     @Override
//     public void updateInputs(CoralArmIOInputs inputs) {
//         BaseStatusSignal.refreshAll(voltageSignal, currentSignal, positionSignal, velocitySignal, temperatureSignal);

//         inputs.motorVoltage = voltageSignal.getValue().in(Volts);
//         inputs.motorCurrent = currentSignal.getValue().in(Amps);
//         inputs.motorPosition = positionSignal.getValue().in(Rotations); // ! might wanna change units later
//         inputs.motorVelocity = velocitySignal.getValue().in(RotationsPerSecond);
//         inputs.motorTemperature = temperatureSignal.getValue().in(Celsius);
    
//     }
// }