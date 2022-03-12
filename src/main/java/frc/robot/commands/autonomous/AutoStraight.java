package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
// import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Intake;

// Pneumatic Dependecies (API)

public class AutoStraight extends CommandBase {
  Drivebase m_db;
  Intake m_toggleIntakeSystem;

  double m_InitHeading;
  double m_speed;
  double m_feet;
  boolean m_isDone = false;
  public AutoStraight(Drivebase db, Intake intakeSystem, double speed) {
    
    m_db = db;
    m_toggleIntakeSystem = intakeSystem;
    m_speed = speed;
    // m_toggleIntakeSystem = intakeSystem;

    addRequirements(db);
    addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_InitHeading = m_db.getAngle();
   // m_db.resetEncoders();
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
   // ForTesting();
    // m_toggleIntakeSystem.autoIntakeSystemOn();
    // while (m_toggleIntakeSystem.getAutoStatus() == true)
    // {
    // for (int i = 0; i < 10; i++)


    m_toggleIntakeSystem.setMotor(0.5);
    m_db.moveForward(-m_speed, -m_db.getAngle());
    // }


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
