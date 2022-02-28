package frc.robot.commands;

import frc.robot.subsystems.pneumatics.SolenoidsIntakeSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ToggleIntakeArm extends CommandBase {
  private final SolenoidsIntakeSystem m_solenoids;
  private boolean m_isDone;

  public ToggleIntakeArm(SolenoidsIntakeSystem subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_solenoids.getArmStatus() == false) {
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
