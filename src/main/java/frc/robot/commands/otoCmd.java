package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;

public class otoCmd extends CommandBase {
  Superstructure m_super;
  public otoCmd(Superstructure m_supers) {
    this.m_super = m_supers;
    addRequirements(m_super);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_super.drive.arcadeDrive(-0.7, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
