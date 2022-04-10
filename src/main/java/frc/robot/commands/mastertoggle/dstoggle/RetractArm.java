package frc.robot.commands.mastertoggle.dstoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Solenoids;

public class RetractArm extends CommandBase {
  private final Solenoids m_solenoids;
  private boolean m_isDone;

  public RetractArm(Solenoids subsystem) {
    m_solenoids = subsystem;
    m_isDone = false;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_solenoids.retractArms();
  
    m_isDone = true;
  
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
