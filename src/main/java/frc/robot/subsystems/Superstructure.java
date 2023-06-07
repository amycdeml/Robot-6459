package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  public WPI_TalonFX solMaster = new WPI_TalonFX(5);
  public WPI_TalonFX solSlave0 = new WPI_TalonFX(4);
  public WPI_TalonFX solSlave1 = new WPI_TalonFX(6);
  public WPI_TalonFX sagMaster = new WPI_TalonFX(2);
  public WPI_TalonFX sagSlave0 = new WPI_TalonFX(1);
  public WPI_TalonFX sagSlave1 = new WPI_TalonFX(3);

  public DifferentialDrive drive = new DifferentialDrive(solMaster, sagMaster);

  public WPI_TalonFX elevatorMotor = new WPI_TalonFX(8);
  public WPI_TalonFX armMotor = new WPI_TalonFX(9);
  public WPI_TalonFX headMotor = new WPI_TalonFX(10);
  public WPI_TalonFX intakeMotor = new WPI_TalonFX(11);

  // public CANSparkMax elevatorNeo = new CANSparkMax(8, MotorType.kBrushless);
  // public RelativeEncoder elevatorNeoEncoder;
  // public SparkMaxPIDController elevatorNeoPID = elevatorNeo.getPIDController();

  public AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  public Superstructure() {

    // elevatorNeo.restoreFactoryDefaults();
    // elevatorNeo.setIdleMode(IdleMode.kBrake);
    // elevatorNeoEncoder = elevatorNeo.getEncoder();
    // elevatorNeoEncoder.setPosition(0);
    // elevatorNeoPID.setP(0.0001);
    // elevatorNeoPID.setOutputRange(-1, 1);
    // elevatorNeoPID.setSmartMotionMaxVelocity(100000, 0);
    // elevatorNeoPID.setSmartMotionMaxAccel(100000, 0);
    // elevatorNeoPID.setSmartMotionAllowedClosedLoopError(1, 0);

    solMaster.configFactoryDefault();
    solSlave0.configFactoryDefault();
    solSlave1.configFactoryDefault();
    sagMaster.configFactoryDefault();
    sagSlave0.configFactoryDefault();
    sagSlave1.configFactoryDefault();
    elevatorMotor.configFactoryDefault();
    headMotor.configFactoryDefault();
    armMotor.configFactoryDefault();
    intakeMotor.configFactoryDefault();


    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    headMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    gyro.reset();
    gyro.calibrate();

    solSlave0.follow(solMaster);
    solSlave1.follow(solMaster);
    sagSlave0.follow(sagMaster);
    sagSlave1.follow(sagMaster);

    solMaster.setInverted(true);
    solSlave0.setInverted(true);
    solSlave1.setInverted(true);
    elevatorMotor.setInverted(true);

    elevatorMotor.configNeutralDeadband(0.001, 0);
    armMotor.configNeutralDeadband(0.001, 0);
    headMotor.configNeutralDeadband(0.001, 0);

    elevatorMotor.setSelectedSensorPosition(0);
    armMotor.setSelectedSensorPosition(0);
    headMotor.setSelectedSensorPosition(0);

    elevatorMotor.configAllowableClosedloopError(0, 50, 1);
    armMotor.configAllowableClosedloopError(0, 50, 1);
    headMotor.configAllowableClosedloopError(0, 50, 1);

    elevatorMotor.config_kP(0, 0.1);
    armMotor.config_kP(0, 0.5); // 0.3
    headMotor.config_kP(0, 0.1);

    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    headMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    headMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

    elevatorMotor.configNominalOutputForward(0, 0);
    elevatorMotor.configNominalOutputReverse(0, 0);
    elevatorMotor.configPeakOutputForward(1, 0);
    elevatorMotor.configPeakOutputReverse(-1, 0);
    armMotor.configNominalOutputForward(0, 0);
    armMotor.configNominalOutputReverse(0, 0);
    armMotor.configPeakOutputForward(0.45, 0);
    armMotor.configPeakOutputReverse(-0.45, 0);
    headMotor.configNominalOutputForward(0, 0);
    headMotor.configNominalOutputReverse(0, 0);
    headMotor.configPeakOutputForward(0.45, 0);
    headMotor.configPeakOutputReverse(-0.45, 0);

    elevatorMotor.configMotionCruiseVelocity(400000, 0);
    elevatorMotor.configMotionAcceleration(60000, 0);
    armMotor.configMotionCruiseVelocity(15000, 0);
    armMotor.configMotionAcceleration(2500, 0); 
    headMotor.configMotionCruiseVelocity(8000, 0);
    headMotor.configMotionAcceleration(24000, 0);

    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    solMaster.configOpenloopRamp(0.375);
    sagMaster.configOpenloopRamp(0.375);

    solMaster.setNeutralMode(NeutralMode.Brake);
    solSlave0.setNeutralMode(NeutralMode.Coast);
    solSlave1.setNeutralMode(NeutralMode.Coast);
    sagMaster.setNeutralMode(NeutralMode.Brake);
    sagSlave0.setNeutralMode(NeutralMode.Coast);
    sagSlave1.setNeutralMode(NeutralMode.Coast);  
  }

  @Override
  public void periodic() {
  }

  static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
