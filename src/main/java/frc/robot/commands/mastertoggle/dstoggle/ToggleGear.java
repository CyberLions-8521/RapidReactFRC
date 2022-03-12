package frc.robot.commands.mastertoggle.dstoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Intake;

public class ToggleGear extends CommandBase {
  private final Intake m_solenoids;
  private boolean m_isDone;

  public ToggleGear(Intake subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_solenoids.getTransmissionStatus() == false) {
      m_solenoids.setGearOn();
    } else {
      m_solenoids.setGearOff();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
