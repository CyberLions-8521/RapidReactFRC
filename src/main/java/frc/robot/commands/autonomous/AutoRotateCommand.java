package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.Constants.DriveConstants;



public class AutoRotateCommand extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;

  double currentAngle;
  double targetAngle;
  double turn;

  public AutoRotateCommand(Drivebase db, double angle) {
   
    m_db = db;
    turn = angle;
    
    addRequirements(db);
    
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    currentAngle = m_db.getAngle(); // Comment this if this does not work - Thien
    targetAngle = currentAngle + turn;
  }

  @Override
  public void execute() {
    currentAngle=m_db.getAngle();
    if(currentAngle<targetAngle)
    {
      m_db.turnInPlace(-0.35);
    } 
    else if (currentAngle> targetAngle)
    {
      m_db.turnInPlace(0.35);
    }

  }
  
  @Override
  public boolean isFinished() {
    return (Math.abs(currentAngle-targetAngle)<1);
  }
}
