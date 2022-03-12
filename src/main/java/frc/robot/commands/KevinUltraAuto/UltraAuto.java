package frc.robot.commands.KevinUltraAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;

// Pneumatic Dependecies (API)

public class UltraAuto extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_index;
  Turret m_turret; 

  double m_InitHeading;
  double m_speed;
  double m_feet;

  boolean m_isDone = false;

  public UltraAuto(Drivebase db, MasterSubsystem index, double speed, double feet) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_index = index;
    m_speed = speed;
    m_feet = feet;
    // m_toggleIntakeSystem = intakeSystem;

    addRequirements(db);
    // addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_InitHeading = m_db.getAngle();
    m_db.resetEncoders();
  }

  // public void ForTesting(){
  //   if (m_feet > m_db.getAverageEncoderDistance()){
  //     m_db.moveForward(-m_speed, -m_db.getAngle());

  //   } else {
  //     m_db.autoArcade(0, 0);
  //     m_isDone = true;
  //   }
  //}

  @Override
  public void execute() {
    if (m_feet > m_db.getAverageEncoderDistance()){
      m_db.autoArcade(-m_speed, 0);
    } else {
      m_db.autoArcade(0, 0);
      m_turret.setSpeed();
    }

 //   m_db.moveForward(-m_speed, -m_db.getAngle());

  }  

  // m_db.moveForward(-0.2);
  // NOTE: may or may not be positive not sure
  // m_db.moveForward(-m_speed, -m_db.getAngle()); //comment or check this out
  // later

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
