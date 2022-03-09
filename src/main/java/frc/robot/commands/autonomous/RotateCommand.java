package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;

public class RotateCommand extends CommandBase {
  /** Creates a new RotateCommand. */
  Drivebase m_db;
  double m_currentAngle;
  double m_targetAngle;
  double m_turn;

  public RotateCommand(Drivebase db, double angle) {
    m_db = db;
    m_turn = angle;
    addRequirements(db);
  }

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
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_currentAngle - m_targetAngle) < 1);
  }
}