package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.IndexSubsystem;

public class LowerIndexor extends CommandBase {
  private final IndexSubsystem m_lowIndexMotor;
  private boolean m_returnStatus;

  public LowerIndexor(IndexSubsystem subsystem) {
    m_lowIndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_lowIndexMotor.getLowerIndexStatus() == false) {
      m_lowIndexMotor.lowerIndexOn();

    } else {
      m_lowIndexMotor.lowerIndexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
