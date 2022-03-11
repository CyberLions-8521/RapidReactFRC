
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.commands.mastertoggle.Shoot;
import frc.robot.RobotContainer;

public class AutoIntakeSystem extends CommandBase {

  Turret m_toggleIntakeSystem;

  public AutoIntakeSystem(Turret subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_toggleIntakeSystem = subsystem;

    // m_toggleIntakeSystem = intakeSystem;

    addRequirements(subsystem);
    // addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    // m_toggleIntakeSystem.extendArms();
    // m_toggleIntakeSystem.autointakeOn();
    

  }



  
  
  @Override
  public void execute() {

  
    m_toggleIntakeSystem.setSpeed();

  
    

    // m_db.moveForward(-0.2);
    // NOTE: may or may not be positive not sure

  }

  @Override
  public void end(boolean interrupted) {
    //m_toggleIntakeSystem.autoIntakeSystemOff();
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
