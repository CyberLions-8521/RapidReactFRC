package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;



public class AutoTurretIndex extends CommandBase {
  /** Creates a new RotateCommand. */
  Drivebase m_db;
  Turret m_tb;
  MasterSubsystem m_id;
  double m_currentAngle;
  double m_targetAngle;
  double m_turn;

  public AutoTurretIndex(Drivebase db, Turret tb, MasterSubsystem id, double angle) {
    m_db = db;
    m_tb = tb;
    m_id = id;
    m_turn = angle;
    addRequirements(db);
    addRequirements(id);
    addRequirements(tb);
  }
  private final int targetSpeed = 500;
  private final int maxtargetSpeed = 600;

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_currentAngle = m_db.getAngle();
    m_targetAngle = m_currentAngle + m_turn;
  }

  @Override
  public void execute() {
    m_currentAngle = m_db.getAngle();
    if (m_currentAngle < m_targetAngle) {
      m_db.turnInPlace(-0.35);
    } else if (m_currentAngle > m_targetAngle) {
      m_db.turnInPlace(0.35);
    }
    
    if (m_tb.getShooterStatus() == false && m_id.getIndexStatus() == false
        && m_tb.getSpeed() > targetSpeed && targetSpeed < maxtargetSpeed) {
          m_tb.setSpeed(565);
      m_id.indexOn();
    } else {
      m_tb.setSpeed(0);
      m_id.indexOff();
    }


    //Toggle comments

  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_currentAngle - m_targetAngle) < 1);
  }
}