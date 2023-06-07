// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private double turretKConv = -1055.55;
  private double turretMinPos = -548000; // 240000
  private double turretMaxPos = 548000; // -240000

  public WPI_TalonFX turretMotor = new WPI_TalonFX(7);

  /** Creates a new Turret. */
  public Turret() {
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    turretMotor.configNeutralDeadband(0.001, 0);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.configAllowableClosedloopError(0, 50, 1);
    turretMotor.config_kP(0, 0.1);
    turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    turretMotor.configNominalOutputForward(0, 0);
    turretMotor.configNominalOutputReverse(0, 0);
    turretMotor.configPeakOutputForward(1, 0);
    turretMotor.configPeakOutputReverse(-1, 0);
    turretMotor.configMotionCruiseVelocity(45000, 0); // 30000
    turretMotor.configMotionAcceleration(30000, 0); // 15000

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurret(boolean enabled, double turretDegrees) {
    double calculatedAngle = turretKConv * turretDegrees;
    if (enabled) {
      turretMotor.set(ControlMode.MotionMagic,
          MathUtil.clamp(calculatedAngle, turretMinPos, turretMaxPos));
      if (calculatedAngle > turretMaxPos) {
        System.out.println("Turret MAX is trying to get out of bounds!");
      } else if (calculatedAngle < turretMinPos) {
        System.out.println("Turret MIN is trying to get out of bounds!");
      }
    } else {
      turretMotor.stopMotor();
    }
  }

  public double getTurretPos() {
    double turretPos = (turretMotor.getSelectedSensorPosition() + 0.001)
        / turretKConv;
    return turretPos;
  }
}
