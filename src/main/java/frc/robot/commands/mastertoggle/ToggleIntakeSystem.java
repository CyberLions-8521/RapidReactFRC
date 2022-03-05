package frc.robot.commands.mastertoggle;






import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class ToggleIntakeSystem extends CommandBase {
  private final MasterSubsystem m_intakeSystem;
  private boolean m_isDone;

  public ToggleIntakeSystem(MasterSubsystem subsystem) {
    m_intakeSystem = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_intakeSystem.getGearStatus() == 1) {
      m_intakeSystem.setGear1();
    } else {
      m_intakeSystem.setGear2();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
