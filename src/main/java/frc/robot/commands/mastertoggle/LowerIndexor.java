package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class LowerIndexor extends CommandBase {
  private final MasterSubsystem m_lowIndexMotor;
  private boolean m_returnStatus;

  public LowerIndexor(MasterSubsystem subsystem) {
    m_lowIndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_lowIndexMotor.getLowerIndexStatus() == false) {
      m_lowIndexMotor.lowerIndexOn();
    } else {
      m_lowIndexMotor.IndexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public void initialize() {
    if (m_motor.getLowerIndexStatus() == false) {
      m_motor.lowerIndexOn();

    } else {
      m_motor.indexOff();
    }
    m_returnStatus = true;
  }

  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
