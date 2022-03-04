package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ToggleIntakeAuto extends CommandBase {
  private final MasterSubsystem m_solenoids;
 
  private boolean m_isDone;

  public ToggleIntakeAuto(MasterSubsystem subsystem) {
    m_solenoids = subsystem;
   
    m_isDone = false;
    addRequirements(subsystem);
  }
  @Override
  public void initialize() {
    if (m_solenoids.getAutoStatus() == false) {
      
      m_solenoids.AutoIntakeSystemON();
    } else {
    
        m_solenoids.AutoIntakeSystemOFF();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
