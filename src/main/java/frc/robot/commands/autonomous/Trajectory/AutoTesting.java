


package frc.robot.commands.autonomous.Trajectory;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
// import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Intake;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.RobotContainer;

public class AutoTesting extends CommandBase {
  Drivebase m_db;
  Intake m_master;
  Turret m_turret;

  double m_InitHeading;
  double m_speed;

  public AutoTesting(Drivebase db, Intake master, Turret turret, double speed) {
   
    m_db = db;
    m_speed = speed;
    m_master = master;
    m_turret = turret;

    addRequirements(db);
    addRequirements(master);
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_InitHeading = m_db.getAngle();
  }

  @Override
  public void execute() {
  
    m_master.autoIntakeSystemOn();
    // m_master.setMotor(0.6);
    // m_turret.setSpeed();
    
    // m_db.moveForward(-0.2);
    // NOTE: may or may not be positive not sure
   //m_db.autoArcade(m_speed, 0); This could be the reason by bot wasnt moving when called double check
   m_db.moveForward(-m_speed, -m_db.getAngle()); // Moving Straight to shooot
  }

  @Override
  public void end(boolean interrupted) {}
;  
  @Override
  public boolean isFinished() {
    return false;
  }
}
