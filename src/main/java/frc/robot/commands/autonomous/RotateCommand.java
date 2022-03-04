package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.dreadsubsystem.Drivebase;

public class RotateCommand extends CommandBase {
  /** Creates a new RotateCommand. */
  Drivebase m_db;
  double currentAngle;
  double targetAngle;
  double turn;
  public RotateCommand(Drivebase db, double angle) {
    m_db = db;
    turn = angle;
    addRequirements(db);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_db.getGyro().reset();
    currentAngle = m_db.getAngle();
    targetAngle = currentAngle + turn;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    currentAngle = m_db.getAngle();
    if (currentAngle < targetAngle)
    {
      m_db.turnInPlace(-0.35);
    }
    else if (currentAngle > targetAngle)
    {
      m_db.turnInPlace(0.35);
    }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(currentAngle-targetAngle) < 1);
  }
}