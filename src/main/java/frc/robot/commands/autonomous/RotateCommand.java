package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.Constants.DriveConstants;

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

public class RotateCommand extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;

  double currentAngle;
  double targetAngle;
  double turn;

  public RotateCommand(Drivebase db, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    turn = angle;
    
    addRequirements(db);
    
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
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
