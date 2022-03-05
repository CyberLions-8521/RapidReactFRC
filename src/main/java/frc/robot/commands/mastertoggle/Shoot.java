package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Turret;

import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class Shoot extends CommandBase {
  private final Turret m_shooter;
  private final MasterSubsystem m_index;
  private boolean m_isDone;

  public Shoot(Turret shooter, MasterSubsystem index) {
    m_shooter = shooter;
    m_index = index;
    m_isDone = false;
    addRequirements(shooter);
    addRequirements(index);
  }

  private final int targetSpeed = 500;
  private final int maxtargetSpeed = 600;

  @Override
  public void initialize() {
    if (m_shooter.getShooterStatus() == false && m_index.getIndexStatus() == false
        && m_shooter.getSpeed() > targetSpeed && targetSpeed < maxtargetSpeed) {
      m_shooter.setSpeed(565);
      m_index.indexOn();
    } else {
      m_shooter.setSpeed(0);
      m_index.indexOff();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}