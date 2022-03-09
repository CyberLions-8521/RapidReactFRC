package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class toggleIndexSystem extends CommandBase {
  private final MasterSubsystem m_lowIndexMotor;
  private boolean m_returnStatus;

  public toggleIndexSystem(MasterSubsystem subsystem) {
    m_lowIndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_lowIndexMotor.getIndexStatus() == false) {
      m_lowIndexMotor.indexOn();

    } else {
      m_lowIndexMotor.indexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
