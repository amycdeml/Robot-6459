package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autocommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  // subsystems
  Superstructure m_Superstructure = new Superstructure();
  Turret m_turret = new Turret();

  autocommand m_auto = new autocommand(m_Superstructure);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return m_auto;
  }
}
