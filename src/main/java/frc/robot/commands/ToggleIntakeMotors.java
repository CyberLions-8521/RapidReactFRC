package frc.robot.commands;
import frc.robot.subsystems.ToggleSystems.ToggleGeneralMotors;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeMotors extends CommandBase {
  private final ToggleGeneralMotors m_motorsUpper;
  private boolean m_isDone;

  public ToggleIntakeMotors(ToggleGeneralMotors subsystem) {
    m_motorsUpper = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_motorsUpper.getIntakeStatus() == false) {
        m_motorsUpper.IntakeOn();
    } else {
        m_motorsUpper.IntakeOff();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
