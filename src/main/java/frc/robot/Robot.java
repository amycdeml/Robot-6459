package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  ShuffleboardTab tab = Shuffleboard.getTab("Sensor_Durumlari");

  GenericEntry armMotorPos = tab.add("armMotorPos", 0).getEntry();
  GenericEntry headMotorPos = tab.add("headMotorPos", 0).getEntry();
  GenericEntry turretMotorPos = tab.add("turretMotorPos", 0).getEntry();
  GenericEntry elevatorMotorPos = tab.add("elevatorMotorPos", 0).getEntry();
  GenericEntry armMotorTemp = tab.add("armMotorTemp", 0).getEntry();
  GenericEntry headMotorTemp = tab.add("headMotorTemp", 0).getEntry();
  GenericEntry turretMotorTemp = tab.add("turretMotorTemp", 0).getEntry();
  GenericEntry elevatorMotorTemp = tab.add("elevatorMotorTemp", 0).getEntry();
  GenericEntry armMotorCurrent = tab.add("armMotorCurrent", 0).getEntry();
  GenericEntry headMotorCurrent = tab.add("headMotorCurrent", 0).getEntry();
  GenericEntry turretMotorCurrent = tab.add("turretMotorCurrent", 0).getEntry();
  GenericEntry elevatorMotorCurrent = tab.add("elevatorMotorCurrent", 0).getEntry();
  GenericEntry gyroPos = tab.add("gyroPos", 0).getEntry();

  public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  Compressor c = new Compressor(PneumaticsModuleType.CTREPCM);
  AddressableLED led = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(78);

  Joystick driverJoystick = new Joystick(0);
  Joystick yardimciJoystick = new Joystick(1);

  Timer backlashTimer = new Timer();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  public static double x = 0;
  public static double y = 0;
  public static double targetStatus = 0;
  public static double targetArea = 0;
  double minTargetArea = 0;

  int curretPose = 0;

  double elevatorKConv = 9090.90;
  double armKConv = 354.6;

  private double elevatorMinPos = -255000;
  private double elevatorMaxPos = 0;
  private double armMinPos = -500;
  private double armMaxPos = 40000;

  double kDriveSpeed = 0.87;
  double kRotationSpeed = 0.56;

  double minXLimiter = -20;
  double maxXLimiter = 20;

  int status;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    status = 0;
    m_robotContainer.m_Superstructure.gyro.reset();
    CameraServer.startAutomaticCapture();
  }

  void ledRenk() {
    Alliance ally = DriverStation.getAlliance();
    for (var id = 0; id < 78; id++) {
      if (id < 42 && ally.equals(Alliance.Red)) {
        ledBuffer.setRGB(id, 255, 0, 0);
      } else if (id < 42 && ally.equals(Alliance.Blue)) {
        ledBuffer.setRGB(id, 0, 0, 255);
      } else if (id > 42 && id < 79) {
        variableled(id);
      }
    }
  }

  void variableled(int id) {
    if (yardimciJoystick.getRawButton(11)) {
      ledBuffer.setRGB(id, 255, 255, 0); // SARI
      status = 2;
    } else if (yardimciJoystick.getRawButton(12)) {
      ledBuffer.setRGB(id, 100, 0, 150); // MOR
      status = 1;
    }
  }

  @Override
  public void robotPeriodic() {
    System.out.println(getHeadPos());
    CommandScheduler.getInstance().run();
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    targetStatus = tv.getDouble(0.0);
    targetArea = ta.getDouble(0.0);

    ledRenk();
    led.setLength(78);
    led.setData(ledBuffer);
    led.start();

    c.enableDigital();

    elevatorMotorPos.setDouble(m_robotContainer.m_Superstructure.elevatorMotor.getSelectedSensorPosition());
    armMotorPos.setDouble(m_robotContainer.m_Superstructure.armMotor.getSelectedSensorPosition());
    headMotorPos.setDouble(m_robotContainer.m_Superstructure.headMotor.getSelectedSensorPosition());
    turretMotorPos.setDouble(m_robotContainer.m_turret.turretMotor.getSelectedSensorPosition());

    elevatorMotorTemp.setDouble(m_robotContainer.m_Superstructure.elevatorMotor.getTemperature());
    armMotorTemp.setDouble(m_robotContainer.m_Superstructure.armMotor.getTemperature());
    headMotorTemp.setDouble(m_robotContainer.m_Superstructure.headMotor.getTemperature());
    turretMotorTemp.setDouble(m_robotContainer.m_turret.turretMotor.getTemperature());

    elevatorMotorCurrent.setDouble(m_robotContainer.m_Superstructure.elevatorMotor.getSupplyCurrent());
    armMotorCurrent.setDouble(m_robotContainer.m_Superstructure.armMotor.getSupplyCurrent());
    headMotorCurrent.setDouble(m_robotContainer.m_Superstructure.headMotor.getSupplyCurrent());
    turretMotorCurrent.setDouble(m_robotContainer.m_turret.turretMotor.getSupplyCurrent());

    gyroPos.setDouble(m_robotContainer.m_Superstructure.gyro.getAngle());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    backlashTimer.start();

    m_robotContainer.m_Superstructure.drive.arcadeDrive(driverJoystick.getRawAxis(1) * kDriveSpeed,
        driverJoystick.getRawAxis(2) * kRotationSpeed);

    if (yardimciJoystick.getRawButton(4)) {
      curretPose = 3; // drop position 3
    } else if (yardimciJoystick.getRawButton(1)) {
      curretPose = 2; // drop position 2
    } else if (yardimciJoystick.getRawButton(2)) {
      curretPose = 1; // lower intake position
    } else if (yardimciJoystick.getRawButton(3)) {
      curretPose = 0; // normal position
    }

    if (yardimciJoystick.getRawButton(5)) {
      intakeSolenoid.set(Value.kReverse);
    } else if (yardimciJoystick.getRawButton(6)) {
      intakeSolenoid.set(Value.kForward);
    }

    if (yardimciJoystick.getRawButton(8)) {
      m_robotContainer.m_Superstructure.intakeMotor.set(0.2);
    } else if (yardimciJoystick.getRawButton(7)) {
      if (curretPose == 3 && status == 1) { // drop cube 3
        m_robotContainer.m_Superstructure.intakeMotor.set(-0.7);
      } else if (curretPose == 3 && status == 2) { // drop cone 3
        m_robotContainer.m_Superstructure.intakeMotor.set(-1);
      } else if (curretPose == 2 && status == 1) { // drop cube 2
        m_robotContainer.m_Superstructure.intakeMotor.set(-0.7);
      } else if (curretPose == 2 && status == 2) { // drop cone 2
        m_robotContainer.m_Superstructure.intakeMotor.set(-1);
      } else if (status == 1) { // drop cube 1
        curretPose = 5;
        if (m_robotContainer.m_turret.getTurretPos() > -40 && m_robotContainer.m_turret.getTurretPos() < 40) {
          m_robotContainer.m_Superstructure.intakeMotor.stopMotor();
        } else {
          m_robotContainer.m_Superstructure.intakeMotor.set(-1);
        }
      } else if (status == 2) { // drop cone 1
        curretPose = 4;
        if (m_robotContainer.m_turret.getTurretPos() > -40 && m_robotContainer.m_turret.getTurretPos() < 40) {
          m_robotContainer.m_Superstructure.intakeMotor.stopMotor();
        } else {
          m_robotContainer.m_Superstructure.intakeMotor.set(-0.5);
        }
      }
    } else {
      m_robotContainer.m_Superstructure.intakeMotor.set(0.08);
    }

    if (yardimciJoystick.getRawButton(10)) {
      m_robotContainer.m_Superstructure.gyro.reset();
    }

    boolean visionLockingStatus;
    if (yardimciJoystick.getPOV() == 270 && (curretPose == 2 || curretPose == 3)) {
      visionLockingStatus = true;
    } else {
      visionLockingStatus = false;
      visionIncrement = 0;
    }

    if (getArmPos() > 50) {
      kDriveSpeed = 0.47;
      kRotationSpeed = 0.46;
    } else {
      kDriveSpeed = 0.87;
      kRotationSpeed = 0.56;
    }

    if (backlashTimer.get() > 2) {

      if (curretPose == 0) { // take cone and cube
        setElevator(true, -0.5);
        if (getElevatorPos() > -5) {
          setArm(true, -5);
          setHead(true, 12000);
          if (getArmPos() < 20) {
            m_robotContainer.m_turret.setTurret(true, 0);
          }
        }
        lockOnTarget(false, false);
      } else if (curretPose == 1) { // drop cone and cube 1
        if (getArmPos() > 40) {
          setArm(true, 22.5);
          setHead(true, 8500);
          if (getArmPos() < 25) {
            m_robotContainer.m_turret.setTurret(true, 180);
            if (m_robotContainer.m_turret.getTurretPos() > 170 && m_robotContainer.m_turret.getTurretPos() < 190) {
              if (getArmPos() > 20) {
                setElevator(true, -32.5);
              }
            }
          }
        } else {
          m_robotContainer.m_turret.setTurret(true, 180);
          if (m_robotContainer.m_turret.getTurretPos() > 170 && m_robotContainer.m_turret.getTurretPos() < 190) {
            setArm(true, 22.5);
            setHead(true, 8500);
            if (getArmPos() > 20) {
              setElevator(true, -32.5);
            }
          }
        }
        lockOnTarget(false, false);
      } else if (curretPose == 2) { // drop cone and cube 2
        setElevator(true, -0.5);
        if (getElevatorPos() > -5) {
          lockOnTarget(true, visionLockingStatus);
          setArm(true, 120);
          if (getArmPos() > 30) {
            // setHead(true, -18000);
            setHead(true, setHeadAngle(y));
          }
        }
      } else if (curretPose == 3) { // drop cone and cube 3
        setElevator(true, -0.5);
        if (getElevatorPos() > -5) {
          lockOnTarget(true, visionLockingStatus);
          setArm(true, 120);
          if (getArmPos() > 30) {
            setHead(true, -1000);
          }
        }
      } else if (curretPose == 4) { // drop cone 1
        setElevator(true, -0.5);
        if (getElevatorPos() > -5) {
          setArm(true, -5);
          m_robotContainer.m_turret.setTurret(true, (getTurretAbs() - getTurningOffset()));
          if (getArmPos() < 20) {
            setHead(true, 10500);
          }
        }
        lockOnTarget(false, false);
      } else if (curretPose == 5) { // drop cube 1
        setElevator(true, -0.5);
        if (getElevatorPos() > -5) {
          setArm(true, -5);
          m_robotContainer.m_turret.setTurret(true, (getTurretAbs() - getTurningOffset()));
          if (getArmPos() < 20) {
            // setTurret(true, 0);
            setHead(true, 10500);
          }
        }
        lockOnTarget(false, false);
      }

    } else if (backlashTimer.get() < 1.95) {
      m_robotContainer.m_Superstructure.headMotor.set(ControlMode.PercentOutput, 0.05);
      m_robotContainer.m_Superstructure.armMotor.set(ControlMode.PercentOutput, -0.05);
      m_robotContainer.m_Superstructure.elevatorMotor.set(ControlMode.PercentOutput, 0.05);
      m_robotContainer.m_Superstructure.headMotor.setSelectedSensorPosition(0);
      m_robotContainer.m_Superstructure.armMotor.setSelectedSensorPosition(0);
      m_robotContainer.m_Superstructure.elevatorMotor.setSelectedSensorPosition(0);
    } else {
      m_robotContainer.m_Superstructure.headMotor.stopMotor();
      m_robotContainer.m_Superstructure.armMotor.stopMotor();
      m_robotContainer.m_Superstructure.elevatorMotor.stopMotor();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public double getXVal() {
    double valueToReturn;
    double kMinimizer = 0.05;
    if (x > minXLimiter && x < maxXLimiter) {
      valueToReturn = x;
    } else {
      valueToReturn = 0;
    }
    valueToReturn = valueToReturn * kMinimizer;
    return valueToReturn;
  }

  public void setElevator(boolean enabled, double elevatorCm) {
    double calculatedDistance = elevatorKConv * elevatorCm;
    if (enabled) {
      m_robotContainer.m_Superstructure.elevatorMotor.set(ControlMode.MotionMagic,
          MathUtil.clamp(calculatedDistance, elevatorMinPos, elevatorMaxPos));
    } else {
      m_robotContainer.m_Superstructure.elevatorMotor.stopMotor();
    }
  }

  public double getElevatorPos() {
    double elevatorPos = (m_robotContainer.m_Superstructure.elevatorMotor.getSelectedSensorPosition() + 0.001)
        / elevatorKConv;
    return elevatorPos;
  }

  public void setArm(boolean enabled, double armDegrees) {
    double calculatedAngle = armKConv * armDegrees;
    if (enabled) {
      m_robotContainer.m_Superstructure.armMotor.set(ControlMode.MotionMagic,
          MathUtil.clamp(calculatedAngle, armMinPos, armMaxPos));
    } else {
      m_robotContainer.m_Superstructure.armMotor.stopMotor();
    }
  }

  public double getArmPos() {
    double armPos = (m_robotContainer.m_Superstructure.armMotor.getSelectedSensorPosition() + 0.001) / armKConv;
    return armPos;
  }

  public void setHead(boolean enabled, double rawHeadPos) {
    if (enabled) {
      m_robotContainer.m_Superstructure.headMotor.set(ControlMode.MotionMagic, -rawHeadPos);
    } else {
      m_robotContainer.m_Superstructure.headMotor.stopMotor();
    }
  }

  public double getHeadPos() {
    double headPos = -m_robotContainer.m_Superstructure.headMotor.getSelectedSensorPosition();
    return headPos;
  }

  boolean gyroOffsetLock = false;
  double gyroOffset = 0;
  double visionIncrement = 0;

  public void lockOnTarget(boolean enabled, boolean visionOn) {
    if (enabled) {
      visionIncrement += getXVal();
      double calculatedTurretAngle = (getTurretAbs() - getTurningOffset()) - visionIncrement;
      m_robotContainer.m_turret.setTurret(true, calculatedTurretAngle);
    } else {
      visionIncrement = 0;
    }
  }

  public double getTurningOffset() {
    double valueToReturn;
    if (Math.abs(m_robotContainer.m_Superstructure.gyro.getAngle() % 360) > 180) {
      valueToReturn = -180;
    } else {
      valueToReturn = 180;
    }
    return valueToReturn;
  }

  public double getTurretAbs() {
    double turningOffset = getTurningOffset();
    return (gyroOffset - (-m_robotContainer.m_Superstructure.gyro.getAngle())) +
        turningOffset;
  }

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> HeadAngleMap = new InterpolatingTreeMap<>();

  private static double setHeadAngle(double range) {
    return HeadAngleMap.getInterpolated(new InterpolatingDouble(range)).value;
  }

  public static double[][] HeadAngleValues = {
      { 1.30, -4800 },
      { 3.30, -11500 },
      { 6.08, -16300 }
  };

  static {
    for (double[] pair : HeadAngleValues) {
      HeadAngleMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
  }

  // boolean gyroOffsetLock = false;
  // double gyroOffset = 0;
  // double visionIncrement = 0;

  // public void lockOnTarget(boolean enabled, boolean visionOn) {
  // // System.out.println(getXVal());
  // if (enabled) {
  // if (gyroOffsetLock == false) {
  // gyroOffset = -180;
  // gyroOffsetLock = true;
  // }
  // if (visionOn && getArmPos() > 90) {
  // visionIncrement += getXVal();
  // setTurret(true, getProtectedTurretLockingAngle() - visionIncrement);
  // } else {
  // setTurret(true, getProtectedTurretLockingAngle());
  // }
  // } else {
  // gyroOffsetLock = false;
  // gyroOffset = 0;
  // visionIncrement = 0;
  // }
  // }

  // public double getTurretAbs() {
  // double turningOffset = 180;
  // return (gyroOffset - (-m_robotContainer.m_Superstructure.gyro.getAngle())) +
  // turningOffset;
  // }

  // public double getProtectedTurretLockingAngle() {
  // double valueToReturn;
  // if (getTurretAbs() > 220) {
  // gyroOffset -= 360;
  // valueToReturn = getTurretAbs();
  // } else if (getTurretAbs() < -220) {
  // gyroOffset += 360;
  // valueToReturn = getTurretAbs();
  // } else {
  // valueToReturn = getTurretAbs();
  // }
  // return valueToReturn;
  // }

  // public double getAngleVisionOffset() {
  // double valueToReturn;
  // valueToReturn = getTurretAbs() * 0.01;
  // System.out.print(getTurretAbs());
  // System.out.print(" ");
  // System.out.println(valueToReturn);
  // return valueToReturn;
  // }

}
