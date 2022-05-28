package frc.robot.commands.IndexerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class toggleReverseIndexSystem extends CommandBase {
  private final MasterSubsystem m_reverseIndex;
  private boolean m_returnStatus;

  public toggleReverseIndexSystem(MasterSubsystem subsystem) {
    m_reverseIndex = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_reverseIndex.getLowerIndexStatus() == false && m_reverseIndex.getIndexStatus() == false && m_reverseIndex.getIntakeStatus() == false) {
      m_reverseIndex.reverseIndexOn();

    } else {
      m_reverseIndex.reverseIndexOff();
      m_reverseIndex.indexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
