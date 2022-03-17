package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;

//additional test

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// Pneumatic Dependecies (API)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class AutoStraight extends CommandBase {
  Drivebase m_db;
  //MasterSubsystem m_toggleIntakeSystem;

  double m_InitHeading;
  double m_speed;

  public AutoStraight(Drivebase db, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speed;
   // m_toggleIntakeSystem = intakeSystem;

    addRequirements(db);
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
  //  if (m_toggleIntakeSystem.getAutoStatus() == true)
  //  {
  //   m_db.moveForward(-m_speed, -m_db.getAngle());
  //  }
   


    // m_db.moveForward(-0.2);
    // NOTE: may or may not be positive not sure
   m_db.moveForward(-m_speed, -m_db.getAngle()); //comment or check this out later
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
