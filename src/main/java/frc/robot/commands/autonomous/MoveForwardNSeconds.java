package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class MoveForwardNSeconds extends CommandBase {
  /** Creates a new MoveForwardNSeconds. */
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;
  
  double m_InitHeading;
  double m_speed;

  public MoveForwardNSeconds(Drivebase db, MasterSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speed;
    m_toggleIntakeSystem = intake;

    addRequirements(db);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_InitHeading = m_db.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    

    // m_db.moveForward(-0.2);
    // NOTE: may or may not be positive not sure
    m_db.moveForward(-m_speed, -m_db.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
