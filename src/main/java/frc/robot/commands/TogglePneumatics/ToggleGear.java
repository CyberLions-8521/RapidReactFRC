package frc.robot.commands.TogglePneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pneumatics.IntakeSolenoid;

public class ToggleGear extends CommandBase {
  private final IntakeSolenoid m_solenoids;
  private boolean m_isDone;

  public ToggleGear(IntakeSolenoid subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_solenoids.getGearStatus() == 1) {
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
