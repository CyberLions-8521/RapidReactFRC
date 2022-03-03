package frc.robot.commands.dstoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pneumatics.SolenoidsSystem;

public class ToggleGear extends CommandBase {
    private final SolenoidsSystem m_solenoids;
    private boolean m_isDone;
  
    public ToggleGear(SolenoidsSystem subsystem) {
      m_solenoids = subsystem;
      m_isDone = false;
      addRequirements(subsystem);
    }
  
    @Override
    public void initialize() {
      if (m_solenoids.getGearStatus() == 1) {
          m_solenoids.setGear1();
      } else {
          m_solenoids.setGear2();
      }
      m_isDone = true;
    }
  
    @Override
    public boolean isFinished() {
      return m_isDone;
    }
}
