package frc.robot.commands;

import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeArm extends CommandBase {
  private final SolenoidsSystem m_solenoids;
  private boolean m_isDone;

  public ToggleIntakeArm(SolenoidsSystem subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_solenoids.get().equals(DoubleSolenoid.Value.kOff) || m_solenoids.get().equals(DoubleSolenoid.Value.kReverse)) {
        m_solenoids.extendArms();
    } else {
        m_solenoids.retractArms();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
