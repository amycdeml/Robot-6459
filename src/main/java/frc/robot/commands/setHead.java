// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;

public class setHead extends CommandBase {
  /** Creates a new setHead. */
  Superstructure m_super;
  double rawHeadPos;
  public setHead(Superstructure m_superr,double rawHeadPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_super=m_superr;
    this.rawHeadPos = rawHeadPos;
    addRequirements(m_super);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_super.headMotor.set(ControlMode.MotionMagic, -rawHeadPos);
    m_super.intakeMotor.set(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_super.headMotor.set(ControlMode.MotionMagic, -17);
    m_super.intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
