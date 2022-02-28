package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pneumatics.SolenoidsIntakeSystem;

public class ToggleGear extends CommandBase {
  private final SolenoidsIntakeSystem m_solenoids;
  private boolean m_isDone;

  public ToggleGear(SolenoidsIntakeSystem subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_solenoids.getGearStatus() == false) {
      m_solenoids.setGear1();
    } else {
      m_solenoids.setGear2();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
