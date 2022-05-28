package frc.robot.commands.IndexerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class ToggleIndexor extends CommandBase {
  private final MasterSubsystem m_IndexMotor;
  private boolean m_returnStatus;

  public ToggleIndexor(MasterSubsystem subsystem) {
    m_IndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_IndexMotor.getIndexStatus() == false) {
      m_IndexMotor.indexOn();


    } else {
      m_IndexMotor.lowerIndexOff();
      m_IndexMotor.indexOff();
      m_IndexMotor.intakeOff();    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
