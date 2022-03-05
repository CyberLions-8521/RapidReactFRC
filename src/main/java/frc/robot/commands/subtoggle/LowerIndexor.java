package frc.robot.commands.subtoggle;

//imports 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;

public class LowerIndexor extends CommandBase {
  private final ToggleGeneralMotors m_motor;
  private boolean m_returnStatus;

  public LowerIndexor(ToggleGeneralMotors subsystem) {
    m_motor = subsystem;
    addRequirements(subsystem);
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
