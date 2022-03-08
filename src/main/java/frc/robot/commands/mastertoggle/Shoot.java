package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class Shoot extends CommandBase {
  private final Turret m_shooter;
  private final Limelight m_vision;
  private final MasterSubsystem m_index;
  private boolean m_isDone;

  public Shoot(Turret shooter, Limelight vision, MasterSubsystem index) {
    m_shooter = shooter;
    m_vision = vision;
    m_index = index;
    m_isDone = false;
    addRequirements(vision);
    addRequirements(shooter);
    addRequirements(index);
  }

  private final int targetSpeed = 500;
  private final int maxtargetSpeed = 600;

  public void AutoIndexerTele(){
    /*  if(setpoint-50 <  m_encoder.getVelocity()){
             m_index.indexOn();
        } else {
        m_index.indexOff();
       }
      }*/
      
      //For testing
      if(1950 < m_shooter.m_encoder.getVelocity()){
        m_index.indexOn();
      } else {
        m_index.indexOff();
      }
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_shooter.ControllerBindSpeed(RobotContainer.m_controller, 0.5);
    AutoIndexerTele();
    
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }

  public class setDefaultCommand {
  }
}