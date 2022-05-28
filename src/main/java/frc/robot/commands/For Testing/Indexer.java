

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class Indexer extends CommandBase {
  private final MasterSubsystem m_IndexMotor;
  private boolean m_returnStatus;

  public Indexer(MasterSubsystem subsystem) {
    m_IndexMotor = subsystem;
    addRequirements(subsystem);
  }

  

  @Override
  public void execute() {
    m_IndexMotor.indexOn();
 
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
