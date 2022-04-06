package frc.robot.commands.mastertoggle.dstoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class RetractArm extends CommandBase {
  private final MasterSubsystem m_solenoids;
  private boolean m_isDone;

  public RetractArm(MasterSubsystem subsystem) {
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
