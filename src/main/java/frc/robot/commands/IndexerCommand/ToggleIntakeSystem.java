package frc.robot.commands.IndexerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class ToggleIntakeSystem extends CommandBase {
  private final MasterSubsystem m_IndexMotor;
  private boolean m_returnStatus;

  public ToggleIntakeSystem(MasterSubsystem subsystem) {
    m_IndexMotor = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (m_IndexMotor.getLowerIndexStatus() == false && m_IndexMotor.getIndexStatus() == false && m_IndexMotor.getIntakeStatus() == false) {
      m_IndexMotor.lowerIndexOn();
      
      m_IndexMotor.indexOn();
      m_IndexMotor.intakeOn();
     // m_IndexMotor.extendArms();


    } else {
      m_IndexMotor.lowerIndexOff();
      m_IndexMotor.indexOff();
      m_IndexMotor.intakeOff();    }
      m_returnStatus = true;
  }

  @Override
  public void end(boolean interrupted) {
    m_IndexMotor.lowerIndexOff();
    m_IndexMotor.indexOff();
    m_IndexMotor.intakeOff();
  }
  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
