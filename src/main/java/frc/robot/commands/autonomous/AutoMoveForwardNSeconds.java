


package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.RobotContainer;

public class AutoMoveForwardNSeconds extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_master;
  Turret  m_turret;

  double m_InitHeading;
  double m_speed;

  public AutoMoveForwardNSeconds(Drivebase db, MasterSubsystem master, Turret yes, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speed;
    m_master = master;

    m_turret = yes;
   // m_toggleIntakeSystem = intakeSystem;

    addRequirements(db);
    addRequirements(master);
    addRequirements(yes);
    //addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_InitHeading = m_db.getAngle();
  }

  @Override
  public void execute() {
  //  m_toggleIntakeSystem.autoIntakeSystemOn();
  // m_toggleIntakeSystem.setMotor(0.8);
    m_master.autoIntakeSystemOn();
    m_master.setMotor(0.6);
    m_turret.setSpeed();
    
    // m_db.moveForward(-0.2);
    // NOTE: may or may not be positive not sure
    m_db.autoArcade(m_speed, 0);
   // m_db.moveForward(-m_speed, -m_db.getAngle());
  }

  @Override
  public void end(boolean interrupted) {}
;  
  @Override
  public boolean isFinished() {
    return false;
  }
}
