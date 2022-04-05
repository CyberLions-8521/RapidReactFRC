package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class ToggleLowerIndexor extends CommandBase {
  private final MasterSubsystem m_IndexMotor;
  private boolean m_returnStatus;

  public ToggleLowerIndexor(MasterSubsystem subsystem) {
    m_IndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_IndexMotor.getLowerIndexStatus() == false) {
      m_IndexMotor.lowerIndexOn();
    } else {
      m_IndexMotor.lowerIndexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
