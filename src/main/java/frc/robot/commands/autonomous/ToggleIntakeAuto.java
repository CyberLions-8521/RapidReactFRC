package frc.robot.commands.autonomous;

import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ToggleIntakeAuto extends CommandBase {
  private final SolenoidsSystem m_solenoids;
 
  private boolean m_isDone;

  public ToggleIntakeAuto(SolenoidsSystem subsystem) {
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
